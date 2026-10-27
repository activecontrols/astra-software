import serial

PORT = "/dev/ttyACM1"
BAUD_RATE = 921600
PAGE_SIZE = 256
OUTPUT_FILE = "imu_data.bin"


def read_exact(connection, size):
    result = bytearray()

    while len(result) < size:
        chunk = connection.read(size - len(result))
        if not chunk:
            raise TimeoutError(f"Timed out reading {size} bytes from serial port")
        result.extend(chunk)

    return bytes(result)


def checksum(page):
    a = 0
    b = 0

    for value in page:
        a = (a + value) & 0xFF
        b = (b + a) & 0xFF

    return a, b


def main():

    LOG_INTERVAL = 5000
    bytes_processed = 0
    last_bytes_processed = 0
    with serial.Serial(PORT, BAUD_RATE, timeout=1) as connection:
        print(f"Opened {PORT}")

        connection.write(b'dump_flash\n')

        print("Dumping flash")

        with open(OUTPUT_FILE, "wb") as output:
            while True:
                connection.write(b"c")  # Request the next page.

                page = read_exact(connection, PAGE_SIZE)

                bytes_processed += PAGE_SIZE
                received_checksum = tuple(read_exact(connection, 2))

                if checksum(page) != received_checksum:
                    print(checksum(page))
                    print(received_checksum)
                    raise ValueError("Page checksum failed")

                if bytes_processed - last_bytes_processed > LOG_INTERVAL:
                    print(f"Bytes read: {bytes_processed}")
                    last_bytes_processed = bytes_processed

                if (connection.read(1) != b'c'):
                    break

                output.write(page)
        connection.write(b"k")  # Tell the device to stop.

    print(f"Saved raw data to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()