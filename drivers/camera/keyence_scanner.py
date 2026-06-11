"""
Keyence LJ-V7200 Laser Scanner Driver for Hatch.

Implements on-demand profile capture via persistent TCP connection.
The connection is opened once and kept alive. Each call to request_profile()
sends a trigger and reads exactly one profile response. The scanner is silent
between requests. No streaming. No buffer overflow. No continuous consumption.

Architecture: Persistent connection, on-demand request-response.
The connection is scoped to the welding pass, not to individual profiles.

Principle: Event-Driven. Each request is triggered by the application.
Principle: Pure Python. Single-threaded, synchronous I/O.
"""

import socket
import struct
import time
import numpy as np
import logging
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)

# Protocol constants
KEYENCE_FUNDAMENTAL_LENGTH_UNIT = 1e-8      # 0.01 µm → meters
KEYENCE_INVALID_LOWER_BOUND = -524280        # Out-of-range / dead channel threshold

# Trigger command — tells the scanner to capture and return one profile
KEYENCE_TRIGGER_REQUEST = bytes.fromhex(
    "20 00 00 00 01 00 F0 00 00 00 00 00 14 00 00 00 "
    "42 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 "
    "01 01 00 00"
)


class KeyenceScannerDriver:
    """
    Keyence LJ-V7200 on-demand profile capture driver.

    Usage:
        driver = KeyenceScannerDriver("192.168.10.6")
        driver.connect()
        
        # Request profiles as needed
        points, colors = driver.request_profile()
        
        # Or capture a batch at a defined interval
        profiles = driver.capture_profiles(count=200, interval=0.1)
        
        driver.disconnect()
    """

    def __init__(self, ip: str, port: int = 24691):
        """
        Args:
            ip: Scanner controller IP address.
            port: TCP port (default 24691).
        """
        self.ip = ip
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._is_connected = False

    # =================================================================
    # Connection Lifecycle
    # =================================================================

    def connect(self) -> bool:
        """
        Open persistent TCP connection to the scanner.

        The connection stays open until disconnect() is called.
        The scanner is silent between requests.

        Returns:
            True if connected successfully.
        """
        if self._is_connected:
            return True

        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(1.0)
            self._sock.connect((self.ip, self.port))
            self._is_connected = True
            logger.info(f"Connected to Keyence scanner at {self.ip}:{self.port}")
            return True
        except Exception as e:
            self._sock = None
            self._is_connected = False
            logger.error(f"Failed to connect to scanner: {e}")
            return False

    def disconnect(self):
        """Close the TCP connection. Scanner returns to idle."""
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self._is_connected = False
        logger.info("Scanner disconnected")

    def start_streaming(self):
        """Compatibility with CameraPipeline — delegates to connect()."""
        return self.connect()

    def stop_streaming(self):
        """Compatibility with CameraPipeline — delegates to disconnect()."""
        self.disconnect()

    def capture_raw_pointcloud(self):
        """Compatibility with CameraPipeline — delegates to request_profile()."""
        return self.request_profile()

    @property
    def is_connected(self) -> bool:
        """Return whether the scanner is currently connected."""
        return self._is_connected

    # =================================================================
    # Single Profile Request
    # =================================================================

    def request_profile(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Request and receive exactly one profile from the scanner.

        Sends the trigger command, reads the response, unpacks 20-bit
        depth values, filters invalid points, and returns a 3D point cloud.

        The scanner must be connected via connect() before calling this.

        Returns:
            points: (N, 3) float32 in optical frame (meters). May be empty.
            colors: (N, 3) uint8 — blue for scanner visibility.
            Returns (None, None) on error.
        """
        if not self._is_connected or self._sock is None:
            logger.warning("Cannot request profile: not connected")
            return None, None

        try:
            # Send trigger
            self._sock.sendall(KEYENCE_TRIGGER_REQUEST)

            # Read size header
            size_bytes = self._recv_exact(4)
            if len(size_bytes) != 4:
                return None, None

            response_size = struct.unpack("<I", size_bytes)[0]
            if response_size < 84:
                # Short packet — discard and retry
                self._recv_exact(response_size - 4)
                return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

            # Read response body
            data = self._recv_exact(response_size)
            if len(data) < 84:
                return None, None

            # Parse metadata from 84-byte streaming header
            num_profiles = struct.unpack("<H", data[48:50])[0]
            data_unit    = struct.unpack("<H", data[50:52])[0]
            x_start      = struct.unpack("<i", data[52:56])[0]
            x_increment  = struct.unpack("<i", data[56:60])[0]

            # Unpack 20-bit depth values
            profile_bytes = data[84:]
            raw_z = self._unpack_20bit_vectorized(profile_bytes, num_profiles)

            # Filter invalid points (out-of-range flag and dead pixels)
            valid_mask = (raw_z > KEYENCE_INVALID_LOWER_BOUND) & (raw_z != 0)
            valid_z = raw_z[valid_mask]
            valid_indices = np.where(valid_mask)[0]

            n_valid = len(valid_z)
            if n_valid == 0:
                return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)

            # Convert to meters
            depth_unit_m = KEYENCE_FUNDAMENTAL_LENGTH_UNIT * data_unit
            x_unit_m     = KEYENCE_FUNDAMENTAL_LENGTH_UNIT * x_increment
            x_start_m    = KEYENCE_FUNDAMENTAL_LENGTH_UNIT * x_start

            # Build point cloud
            points = np.zeros((n_valid, 3), dtype=np.float32)
            colors = np.zeros((n_valid, 3), dtype=np.uint8)

            points[:, 0] = x_start_m + valid_indices * x_unit_m
            points[:, 1] = 0.0
            points[:, 2] = valid_z * depth_unit_m  # Flip to optical convention
            colors[:] = [0, 0, 255]  # Blue for scanner visibility

            return points, colors

        except socket.timeout:
            logger.debug("Profile request timed out")
            return None, None
        except Exception as e:
            logger.error(f"Profile request error: {e}")
            return None, None

    # =================================================================
    # Batch Capture
    # =================================================================

    def capture_profiles(self,
                         count: int,
                         interval: float = 1.0 / 30) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        Capture exactly `count` profiles at `interval` seconds apart.

        Uses the persistent connection. Sends a request, reads one response,
        waits for the interval, then repeats. The connection must already
        be open (call connect() first).

        Args:
            count: Number of profiles to capture.
            interval: Time between profiles in seconds (e.g., 0.1 for 10 Hz).

        Returns:
            List of (points, colors) tuples. May be shorter than `count`
            if errors occur.
        """
        if not self._is_connected:
            logger.error("Cannot capture batch: not connected")
            return []

        profiles = []

        for i in range(count):
            loop_start = time.time()
            
            points, colors = self.request_profile()
            if points is None:
                break
            if len(points) > 0:
                profiles.append((points, colors))

            # Calculate exact remaining sleep time
            elapsed = time.time() - loop_start
            if elapsed < interval and i < (count - 1):
                time.sleep(interval - elapsed)

        logger.info(f"Batch complete: {len(profiles)}/{count} profiles captured")
        return profiles

    # =================================================================
    # Protocol Helpers
    # =================================================================

    def _recv_exact(self, size: int) -> bytes:
        """Receive exactly `size` bytes, respecting socket timeout."""
        data = b""
        while len(data) < size:
            try:
                chunk = self._sock.recv(size - len(data))
                if not chunk:
                    break
                data += chunk
            except socket.timeout:
                break
        return data

    @staticmethod
    def _unpack_20bit_vectorized(data: bytes, num_points: int) -> np.ndarray:
        """
        Vectorized unpacking of 20-bit signed values from a Little-Endian 
        packed Keyence byte stream.
        """
        blocks_needed = (num_points + 1) // 2
        raw_buffer = np.frombuffer(data[:blocks_needed * 5], dtype=np.uint8)
        raw = raw_buffer.reshape(-1, 5)

        # Cast to 32-bit signed integers safely
        b0 = raw[:, 0].astype(np.int32)
        b1 = raw[:, 1].astype(np.int32)
        b2 = raw[:, 2].astype(np.int32)
        b3 = raw[:, 3].astype(np.int32)
        b4 = raw[:, 4].astype(np.int32)

        # --- CORRECTED LITTLE-ENDIAN SHIFTS ---
        # Point 0: b2 low nibble is the highest 4 bits, b1 is middle, b0 is lowest
        p0 = ((b2 & 0x0F) << 16) | (b1 << 8) | b0

        # Point 1: b4 is the highest 8 bits, b3 is middle, b2 high nibble is lowest 4 bits
        p1 = (b4 << 12) | (b3 << 4) | (b2 >> 4)

        # Sign extension: 20-bit signed → 32-bit signed
        # Check bit 19 (0x80000). If set, subtract 2^20 (0x100000)
        p0 = np.where(p0 & 0x80000, p0 - 0x100000, p0)
        p1 = np.where(p1 & 0x80000, p1 - 0x100000, p1)

        # Interleave: [P0[0], P1[0], P0[1], P1[1], ...]
        result = np.empty(len(p0) * 2, dtype=np.int32)
        result[0::2] = p0
        result[1::2] = p1

        return result[:num_points]

# =====================================================================
# Standalone Test
# =====================================================================

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # 1. Initialize and connect to the scanner
    SCANNER_IP = "192.168.10.6"
    logger.debug(f"Initializing Keyence LJ-V7200 Driver test at {SCANNER_IP}...")
    
    driver = KeyenceScannerDriver(ip=SCANNER_IP)
    
    if not driver.connect():
        logger.debug("CRITICAL: Could not connect to the scanner. Exiting test.")
        exit(1)

    try:
        print("\n>>> Target Check: Ensure an object is placed 100mm–140mm below the sensor head.")
        print(">>> The orange LED on the laser head must be SOLID (not blinking).")
        input("Press Enter when ready to capture a profile...")

        # 2. Capture a single on-demand profile
        print("\nRequesting profile...")
        points, colors = driver.request_profile()

        # 3. Handle data feedback and visualization validation
        if points is None:
            print("ERROR: Failed to retrieve profile data from the socket.")
        elif len(points) == 0:
            print("WARNING: Profile received, but contains 0 valid points.")
            print("The laser head is likely out of range or blinded (empty field of view).")
        else:
            print(f"SUCCESS: Captured profile containing {len(points)} valid surface points!")
            
            # Convert meters to millimeters for standard drafting/welding inspection view
            x_mm = points[:, 0] * 1000.0
            z_mm = points[:, 2] * 1000.0

            print(f"X-Axis Span (Width): {x_mm.min():.2f}mm to {x_mm.max():.2f}mm")
            print(f"Z-Axis Span (Depth): {z_mm.min():.2f}mm to {z_mm.max():.2f}mm")

            # 4. Generate the Matplotlib Plot
            plt.figure(figsize=(10, 5))
            plt.plot(x_mm, z_mm, 'b.', markersize=3, label='Scanned Surface')
            
            plt.title('Keyence LJ-V7200 Profile Capture Test', fontsize=12, fontweight='bold')
            plt.xlabel('Width / X-Axis (mm)', fontsize=10)
            plt.ylabel('Standoff Distance / Z-Axis (mm)', fontsize=10)
            
            # Invert Z-axis so moving the paper closer to the head moves the plot upwards
            plt.gca().invert_yaxis()
            plt.grid(True, linestyle='--', alpha=0.6)
            plt.legend(loc='upper right')
            plt.axis('equal')  # Maintain physical aspect ratio 1:1
            
            print("\nDisplaying plot window. Close the plot window to disconnect the driver safely.")
            plt.show()

    finally:
        # 5. Safe disconnection cleanup pass
        print("\nCleaning up driver hooks...")
        driver.disconnect()
        print("Test complete.")
