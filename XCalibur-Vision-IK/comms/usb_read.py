import serial

ser = serial.Serial('/dev/ttyGS0', 115200, timeout=1)

while True:
    data = ser.read(ser.in_waiting or 1)
    if data:
        print(data.decode('utf-8'), end='')
