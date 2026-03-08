import serial
import sys

if len(sys.argv) < 3:
    print("Error: missing arguments. Example usage: python <python file> <com port> <out file>")

com_port = open(sys.argv[1], "r")
f_out = open(sys.argv[2], "wb")

ser = serial.Serial(sys.argv[1], timeout=None)


# skip this many bytes to start because this is the calibration data
skipcount = 8 * 9 + 12 * 8
def process(b:bytes):
    global skipcount
    if skipcount > 0:
        skipcount -= 1
        f_out.write(b)
        return True
    
    # this byte is a bitfield, decode it and decide how many more bytes until the next flag
    skipcount = 4 + 1 + 16 # read time(4), phase (1), and controller output (16)
    flags = b[0]

    if flags == 255:
        return False # we reached the end of the log

    if flags & 1:
        skipcount += 36 # read sensor data
    
    if (flags >> 1) & 1:
        skipcount += 18 * 4 # read gps data

    f_out.write(b)
    return True

def main():
    ser.write("dump_flash\n")
    ser.flush()
    
    while True:
        for _ in range(255):
            c = ser.read()
            if not process(c):
                ser.write('!')
                ser.flush()
                return
            
        ser.write('c')
        ser.flush()

main()
com_port.close()
f_out.close()
ser.close()