
import argparse
import os
import serial
import time

def wait_for_breaks(
        ser: serial.Serial, break_count, break_char, delay_s=0.2
        ):
    breaks = 0
    break_code = ord(break_char)
    while True:
        data = ser.read_all()
        if breaks >= break_count:
            return
        if data:
            print(f"{str(data.decode())}")
            for b in data:
                if b == break_code:
                    breaks += 1
                if b != break_code and breaks > 0:
                    breaks = 0
        time.sleep(delay_s)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("kpath", type=str)
    parser.add_argument("-p", "--port", default="COM3", type=str)
    parser.add_argument("-b", "--baud", default=115200, type=int)
    args = parser.parse_args()
    ser = serial.Serial(
        args.port, args.baud, timeout=0, parity=serial.PARITY_NONE
        )
    assert ser.is_open, f"Failed to open serial port: {args.port}"
    assert os.path.isfile(args.kpath), f"Given Kernel path is not a valid file: [{args.kpath}]"
    print("Opened port and found Kernel file")
    while True:
        wait_for_breaks(ser, 3, "\x03")
        print("Received 3 breaks. Sending the Kernel")
        with open(args.kpath, mode="rb") as kfile:
            file_size = os.path.getsize(args.kpath)
            print(f"Kernel Size in Bytes: {file_size}")
            # send kernel file size
            ser.write(file_size.to_bytes(length=4, byteorder="little"))
            # then send the kernel file
            kbytes_sent = ser.write(kfile.read())
            assert file_size == kbytes_sent, f"Failed to send the entire kernel"