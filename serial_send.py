import serial
import time

serial_data=serial.Serial('/dev/ttyACM0',9600,timeout=1)
serial_data.flush()

while True:
    serial_data.write(b"forward\n")
    time.sleep(2)
    serial_data.write(b"backward\n")
    time.sleep(2)
