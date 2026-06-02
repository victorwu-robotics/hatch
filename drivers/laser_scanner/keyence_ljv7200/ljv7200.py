"""
LuBan Platform - Keyence LJ-V7200 Line Laser Scanner Driver
Module: luban.drivers.keyence_ljv7200
Version: 1.1
License: MIT
"""

import socket
import struct
import time
import threading
import logging
from typing import Optional

from luban.types.camera import PointCloud
from luban.types.std import Header
from luban.flash import publish  # ← ONLY ZMQ interface needed


KEYENCE_INVALID_DATA_VALUE = 0x7FFFFF00
KEYENCE_DEAD_ZONE_DATA_VALUE = 0x7FFFFF01


class KeyenceLJ7200Driver:
    def __init__(
        self,
        ip: str,
        port: int = 24691,
        frame_id: str = "keyence_ljv7200",
        zmq_pub_endpoint: str = "tcp://127.0.0.1:5556"
    ):
        """
        Initialize the Keyence LJ-V7200 driver.
        
        Args:
            ip: IP address of the scanner controller
            port: TCP port (default 24691)
            frame_id: Frame ID for published messages
        """
        self._ip = ip
        self._port = port
        self._frame_id = frame_id
        self.zmq_pub_endpoint = zmq_pub_endpoint
        
        # Streaming state
        self._streaming_thread = None
        self._streaming_stop_event = None
        self._is_streaming = False
        
        # Logging
        self._logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

    def read_profile(self) -> PointCloud:
        """
        Read a single profile from the Keyence LJ-V7200.
        Assumes the controller is set to continuously send profile data over TCP.
        """
        if self._socket is None:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.connect((self.ip, self.port))
            self._socket.settimeout(2.0)

        # Receive header (16 bytes) + profile data
        # Keyence binary format: [Header:16B][Data: N*8B (X,Z pairs as doubles)]
        header = self._socket.recv(16)
        if len(header) < 16:
            raise RuntimeError("Failed to read header")

        # Interpret header (example: first 4 bytes = profile count, next 4 = point count)
        point_count = int.from_bytes(header[4:8], byteorder='little')
        data_size = point_count * 16  # 2 doubles (X,Z) per point → 16 bytes

        raw_data = self._socket.recv(data_size)
        if len(raw_data) < data_size:
            raise RuntimeError(f"Expected {data_size} bytes, got {len(raw_data)}")

        # Parse as doubles (X, Z pairs)
        points = []
        for i in range(point_count):
            x = struct.unpack_from('<d', raw_data, i * 16)[0]
            z = struct.unpack_from('<d', raw_data, i * 16 + 8)[0]
            # Y is 0 (2D line laser)
            points.append((x, 0.0, z))

        return PointCloud(
            header=Header(stamp=time(), frame_id=self.frame_id),
            points=points
        )

    def start_streaming(self, rate_hz: float) -> None:
        """
        Start streaming laser profiles from the scanner.
        
        Args:
            rate_hz: Target streaming rate in Hz
            
        Raises:
            ConnectionError: If TCP connection fails
            RuntimeError: If already streaming
        """
        if self._is_streaming:
            raise RuntimeError("Already streaming")
            
        '''
        # Test TCP connection before starting thread
        test_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_sock.settimeout(5.0)
        try:
            test_sock.connect((self._ip, self._port))
        except Exception as e:
            test_sock.close()
            raise ConnectionError(f"Failed to connect to {self._ip}:{self._port}: {e}")
        test_sock.close()
        '''

        # Start streaming thread
        self._streaming_stop_event = threading.Event()
        self._streaming_thread = threading.Thread(
            target=self._streaming_worker,
            args=(rate_hz,),
            daemon=True
        )
        self._is_streaming = True
        self._streaming_thread.start()
        self._logger.info("Streaming started")

    def stop_streaming(self) -> None:
        """Stop streaming and close connections."""
        if not self._is_streaming:
            return
            
        self._streaming_stop_event.set()
        if self._streaming_thread:
            self._streaming_thread.join(timeout=2.0)
        self._is_streaming = False
        self._streaming_stop_event = None
        self._streaming_thread = None
        self._logger.info("Streaming stopped")

    @property
    def is_streaming(self) -> bool:
        """Return whether the driver is currently streaming."""
        return self._is_streaming

    def _streaming_worker(self, rate_hz: float):
        """Background worker thread for streaming laser data using request-reply pattern."""
        sleep_interval = 1.0 / rate_hz
        REQUEST_SINGLE_PROFILE = b"\x00\x01\x01\x00"  # Exact command from C++ driver

        FULL_SINGLE_PROFILE_CMD = bytes([
            0x14, 0x00, 0x00, 0x00,  # total size
            0x01, 0x00, 0xF0, 0x00,  # magic
            0x00, 0x00, 0x00, 0x00,  # reserved
            0x04, 0x00, 0x00, 0x00,  # body size
            0x01, 0x00, 0x00, 0x00   # command + padding
        ])

        while not self._streaming_stop_event.is_set():
            start_time = time.time()
            sock = None
            try:
                # Open new connection for each request (Keyence prefers this)
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1.0)
                print(f"🔌 Connecting to {self._ip}:{self._port}...")
                sock.connect((self._ip, self._port))
                
                # Send single profile request
                print("📤 Sending SingleProfile command...")
                # sock.sendall(REQUEST_SINGLE_PROFILE)
                sock.sendall(FULL_SINGLE_PROFILE_CMD)

                # Read header: [reserved:4][point_count:4] (little-endian)
                header = self._recv_exact(sock, 8)
                print(f"📥 Received header: {header.hex()}")
                if len(header) != 8:
                    self._logger.warning("Incomplete header received")
                    continue

                point_count = int.from_bytes(header[4:8], byteorder='little')
                print(f"📊 Point count: {point_count}")
                if point_count <= 0 or point_count > 4096:  # sanity check
                    self._logger.warning(f"Invalid point count: {point_count}")
                    continue

                # Read Z values: point_count × int32 (little-endian)
                z_data = self._recv_exact(sock, point_count * 4)
                if len(z_data) != point_count * 4:
                    self._logger.warning("Incomplete Z data received")
                    continue

                # Parse points
                points = []
                colors = []
                for i in range(point_count):
                    # Unpack Z as signed 32-bit integer
                    z_raw = int.from_bytes(z_data[i*4:(i+1)*4], byteorder='little', signed=True)
                    
                    # Convert to meters (Keyence unit = 1 µm; but Z is scaled by data_unit)
                    # Since we don't read data_unit here, assume 1 unit = 1 µm → divide by 1e6
                    # This matches typical LJ-V7000 output
                    if z_raw in (-999999, -999997):
                        z_m = float('nan')
                    else:
                        z_m = z_raw * 1e-6  # microns → meters

                    # X: assume uniform spacing (e.g., 10 µm per point = 0.00001 m)
                    # You can refine this later by querying x_start/x_increment via GetSetting
                    x_m = i * 10e-6  # 10 µm spacing → adjust based on your head/optics

                    points.append((x_m, 0.0, z_m))
                    colors.append([255, 0, 0])  # Red

                # Publish
                pc = PointCloud(
                    header=Header(stamp=time.time(), frame_id=self._frame_id),
                    points=points,
                    colors=colors
                )
                publish(
                    "sensor/line_scan",
                    pc,
                    bind_address=self.zmq_pub_endpoint
                    )

            except socket.timeout:
                self._logger.debug("Socket timeout during read")
            except OSError as e:
                if e.errno == 113:  # No route to host
                    self._logger.warning("Scanner not responding—check power and program")
                else:
                    self._logger.error(f"Network error: {e}")
            except Exception as e:
                self._logger.error(f"Streaming error: {e}")
            finally:
                if sock:
                    sock.close()

            # Maintain rate
            elapsed = time.time() - start_time
            sleep_time = max(0, sleep_interval - elapsed)
            if sleep_time > 0:
                self._streaming_stop_event.wait(sleep_time)

    def _recv_exact(self, sock, size):
        """Receive exactly `size` bytes from socket."""
        data = b''
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                break
            data += chunk
        return data

    def __del__(self):
        # Just ensure cleanup if resources exist
        if hasattr(self, '_zmq_publisher'):
            self._zmq_publisher.close()
        if hasattr(self, '_zmq_context'):
            self._zmq_context.destroy()