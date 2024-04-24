
import argparse
import os
import serial

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
    assert os.path.isfile(args.kpath), f"Kernel path given is not a valid file: [{args.kpath}]"