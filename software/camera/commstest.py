import serial

ser = serial.Serial('/dev/ttyACM1',115200)


sent = bytearray([132,3,100,100,100])
ser.write(sent)
print(sent)


