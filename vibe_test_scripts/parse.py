import csv
import struct

INPUT_FILE = "imu_data.bin"
OUTPUT_FILE = "imu_data.csv"
MARKER = 0xABCD
RECORD = struct.Struct("<HI3h3h")

ACCEL_RESOLUTION = 1.0 / 4096.0
GYRO_RESOLUTION = 1.0 / 16.4

def main():
    with open(INPUT_FILE, "rb") as binary_file:
        data = binary_file.read()

    columns = [
        "time_us",
        "raw_gyro_x", "raw_gyro_y", "raw_gyro_z",
        "raw_accel_x", "raw_accel_y", "raw_accel_z",
        "gyro_x_deg", "gyro_y_deg", "gyro_z_deg",
        "accel_x_g", "accel_y_g", "accel_z_g"
    ]

    records_written = 0
    offset = 0

    with open(OUTPUT_FILE, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(columns)

        while offset + 2 <= len(data):
            marker = struct.unpack_from("<H", data, offset)[0]
            if marker != MARKER:
                break

            if offset + RECORD.size > len(data):
                raise ValueError(f"Incomplete IMU record at byte offset {offset}")

            _, time_us, *values = RECORD.unpack_from(data, offset)

            gyro_deg = [a * GYRO_RESOLUTION for a in values[0:3]]
            accel_g = [a * ACCEL_RESOLUTION for a in values[3:6]]

            writer.writerow([time_us, *values, *gyro_deg, *accel_g])
            records_written += 1
            offset += RECORD.size

    print(f"Wrote {records_written} records to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()