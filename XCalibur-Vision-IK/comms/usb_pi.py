import serial

ser = serial.Serial('/dev/ttyGS0', 115200)
while True:
    message = ser.readline()
    print("Received:", message.decode('utf-8'))
