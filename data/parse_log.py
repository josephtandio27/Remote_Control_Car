import struct
import csv
import sys
from pathlib import Path

# Matches TelemetryPacket in Telemetry.h (packed)
# uint32_t timestamp
# float accel[3]
# float gyro[3]
# float mag[3]
# float steerAngle
# float pidYaw
PACKET_FORMAT = "<I 3f 3f 3f f f"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

HEADERS = [
    "timestamp_ms",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
    "steer_angle",
    "pid_yaw",
]


def parse(bin_path: Path, csv_path: Path):
    data = bin_path.read_bytes()
    total = len(data)
    n_packets = total // PACKET_SIZE
    leftover = total % PACKET_SIZE

    if n_packets == 0:
        print("No complete packets found — file may be empty or corrupted.")
        return

    print(f"Parsing {n_packets} packets ({PACKET_SIZE} bytes each)...")
    if leftover:
        print(f"Warning: {leftover} trailing bytes ignored (incomplete packet at end)")

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(HEADERS)
        for i in range(n_packets):
            chunk = data[i * PACKET_SIZE : (i + 1) * PACKET_SIZE]
            fields = struct.unpack(PACKET_FORMAT, chunk)
            writer.writerow(fields)

    print(f"Saved to {csv_path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python parse_log.py <log_xxx.bin> [output.csv]")
        sys.exit(1)

    bin_file = Path(sys.argv[1])
    csv_file = Path(sys.argv[2]) if len(sys.argv) >= 3 else bin_file.with_suffix(".csv")

    if not bin_file.exists():
        print(f"File not found: {bin_file}")
        sys.exit(1)

    parse(bin_file, csv_file)
