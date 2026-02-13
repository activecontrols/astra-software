#!/usr/bin/env python3
import argparse
import struct
import sys
import threading
import time
import serial

# make sure this matches cpp

DS = ord('S')
DE = ord('E')
ESC = ord('\\')
R = 0x5A

COMMAND_BUFFER_SIZE = 200
CSUM_FMT = "<i"   # little-endian int32

def make_cmd_packet(command: str) -> bytes:
    buf = bytearray(COMMAND_BUFFER_SIZE)
    b = command.encode("ascii", errors="strict")
    if len(b) >= COMMAND_BUFFER_SIZE:
        raise ValueError("Command too long")
    buf[:len(b)] = b
    return bytes(buf)

def checksum(payload: bytes) -> int:
    return sum(payload) & 0xFFFFFFFF

def csum_encode(payload: bytes) -> bytes:
    csum = checksum(payload)
    if csum >= 0x80000000:
        csum -= 0x100000000
    return payload + struct.pack(CSUM_FMT, csum)

def escape_delimit(raw: bytes) -> bytes:
    out = bytearray()
    out.append(DS)
    for b in raw:
        if b in (DS, DE, ESC):
            out.append(ESC)
            out.append(b ^ R)
        else:
            out.append(b)
    out.append(DE)
    return bytes(out)

def encode_command(cmd: str) -> bytes:
    return escape_delimit(csum_encode(make_cmd_packet(cmd)))


def reader_loop(ser, stop_evt, show_hex_flag):
    while not stop_evt.is_set():
        try:
            data = ser.read(256)
        except Exception as e:
            print(f"\n[serial error] {e}", file=sys.stderr)
            break

        if not data:
            continue

        if show_hex_flag[0]:
            print("\n[RX hex]", " ".join(f"{b:02x}" for b in data))

        print(data.decode("utf-8", errors="replace"), end="", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hex", action="store_true")
    args = ap.parse_args()

    show_hex = [args.hex] # hack to make value shared and update everywhere.

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        ser.reset_input_buffer()

        stop_evt = threading.Event()
        t = threading.Thread(target=reader_loop,
                             args=(ser, stop_evt, show_hex),
                             daemon=True)
        t.start()

        print(f"[connected {args.port} @ {args.baud}]")
        print("Type commands. /quit to exit. /hex toggles hex.")

        try:
            while True:
                line = input("").strip() # no prompt needed tbh.
                if not line:
                    continue
                # print()

                if line == "/quit":
                    break

                if line == "/hex":
                    show_hex[0] = not show_hex[0]
                    print(f"[hex {'ON' if show_hex[0] else 'OFF'}]")
                    continue

                pkt = encode_command(line)
                ser.write(pkt)
                ser.flush()

                if show_hex[0]:
                    print("[TX hex]", " ".join(f"{b:02x}" for b in pkt))

        except (EOFError, KeyboardInterrupt):
            pass
        finally:
            stop_evt.set()
            time.sleep(0.1)

if __name__ == "__main__":
    main()
