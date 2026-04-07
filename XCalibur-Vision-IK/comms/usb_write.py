import serial

ser = serial.Serial('/dev/ttyGS0', 115200)  # change COM3 to your port
ser.write(b'OK Computer!\n')
ser.close()
