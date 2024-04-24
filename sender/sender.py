
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
    assert os.path.isfile(args.kpath), f"Given Kernel path is not a valid file: [{args.kpath}]"

    breaks = 0
    while True:
        data = ser.read_all()
        if breaks >= 3:
            break
        if data:
            for b in data:
                if b == "\x03":
                    breaks += 1
                if b != "\x03" and breaks > 0:
                    breaks = 0
    print("Received 3 breaks. Sending the Kernel")
    with open(args.kpath, mode="rb") as kfile:
        file_size = len(kfile)
        ser.write(file_size.to_bytes())
        kbytes_sent = ser.write(kfile)
        assert file_size == kbytes_sent, f"Failed to send the entire kernel"