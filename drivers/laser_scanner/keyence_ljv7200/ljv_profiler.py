#!/usr/bin/env python3
"""
Keyence LJ-V7200 High-Speed Single Profile Driver
- Verified against C++ reference implementation
- Port: 24691
- Endianness: Little-endian
- Frame: lj_v7200_optical_frame (X=laser line, Z=depth from focal plane)
"""

import socket
import struct
import sys

# Protocol constants
KEYENCE_FUNDAMENTAL_LENGTH_UNIT = 1e-8  # 0.01 µm → meters
KEYENCE_JUDGEMENT_WAIT_DATA_VALUE = -524285

# Exact 36-byte request (little-endian, verified)
KEYENCE_REQUEST = bytes.fromhex(
    "20 00 00 00 01 00 F0 00 00 00 00 00 14 00 00 00 "
    "42 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 "
    "01 01 00 00"
)

def unpack_20bit_signed(data, num_points):
    """Unpack 20-bit signed integers from packed byte stream."""
    points = []
    byte_idx = 0
    i = 0
    while i < num_points and byte_idx + 4 < len(data):
        if byte_idx + 5 > len(data):
            break
        b0, b1, b2, b3, b4 = data[byte_idx:byte_idx+5]
        byte_idx += 5

        # Point 0: 20 bits from b0, b1, b2
        raw0 = (b0 << 16) | (b1 << 8) | b2
        raw0 = (raw0 & 0xFFFFF) - (0x100000 if raw0 & 0x80000 else 0)
        points.append(raw0)
        i += 1
        if i >= num_points:
            break

        # Point 1: lower 4 bits of b2 + b3, b4
        raw1 = ((b2 & 0x0F) << 16) | (b3 << 8) | b4
        raw1 = (raw1 & 0xFFFFF) - (0x100000 if raw1 & 0x80000 else 0)
        points.append(raw1)
        i += 1
    return points[:num_points]

def request_profile(host, port):
    """Send request and parse response."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(10)
        sock.connect((host, port))
        sock.send(KEYENCE_REQUEST)

        # Read response size prefix (4 bytes, little-endian)
        size_bytes = sock.recv(4)
        if len(size_bytes) != 4:
            raise RuntimeError("Failed to read response size")
        response_size = struct.unpack("<I", size_bytes)[0]

        # Read full response
        data = b""
        while len(data) < response_size:
            chunk = sock.recv(response_size - len(data))
            if not chunk:
                raise RuntimeError("Connection closed")
            data += chunk

        # Parse metadata (absolute offsets in response)
        num_profiles = struct.unpack("<H", data[48:50])[0]
        data_unit = struct.unpack("<H", data[50:52])[0]
        x_start = struct.unpack("<i", data[52:56])[0]
        x_increment = struct.unpack("<i", data[56:60])[0]
        trigger_count = struct.unpack("<I", data[64:68])[0]
        encoder_count = struct.unpack("<I", data[68:72])[0]

        # Unpack profile data
        profile_bytes = data[84:]
        profile_points = unpack_20bit_signed(profile_bytes, num_profiles)

        metadata = {
            'num_profiles': num_profiles,
            'data_unit': data_unit,
            'x_start': x_start,
            'x_increment': x_increment,
            'trigger_count': trigger_count,
            'encoder_count': encoder_count,
        }

        return metadata, profile_points

def is_point_valid(pt):
    """Check if point is valid (not out-of-range)."""
    return pt > KEYENCE_JUDGEMENT_WAIT_DATA_VALUE

def display_profile(metadata, profile_points):
    """Display profile metadata and sample points."""
    depth_unit_m = KEYENCE_FUNDAMENTAL_LENGTH_UNIT * metadata['data_unit']
    x_unit_m = KEYENCE_FUNDAMENTAL_LENGTH_UNIT * metadata['x_increment']
    x_start_m = KEYENCE_FUNDAMENTAL_LENGTH_UNIT * metadata['x_start']

    print(f"Encoder count: {metadata['encoder_count']}")
    print(f"Trigger count: {metadata['trigger_count']}")
    print(f"Profile width: {metadata['num_profiles']}")
    print(f"Depth Unit: {depth_unit_m * 1e6:.3f} (um)")
    print(f"Step Unit: {x_unit_m * 1e6:.3f} (um)")
    print(f"Start Offset: {x_start_m * 1e6:.3f} (um)")

    for i in range(0, len(profile_points), 100):
        x_pos = (x_start_m + i * x_unit_m) * 1e6  # µm
        pt = profile_points[i]
        if not is_point_valid(pt):
            print(f"Point{i} ({x_pos:.3f}, INVALID)")
        else:
            z_um = pt * depth_unit_m * 1e6
            print(f"Point{i} ({x_pos:.3f}, {z_um:.3f})")

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 keyence_ljv7000_profile.py <host> <port>", file=sys.stderr)
        print("Example: python3 keyence_ljv7000_profile.py 192.168.10.6 24691")
        sys.exit(1)

    host, port = sys.argv[1], int(sys.argv[2])

    try:
        metadata, profile_points = request_profile(host, port)
        display_profile(metadata, profile_points)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()