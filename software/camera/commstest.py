import serial

ser = serial.Serial('/dev/ttyACM0',115200)

mask_8bit = 0xFF
mask_16bit = 0xFFFF
mask_32bit = 0xFFFFFFFF

type_select = 0b10000000 & mask_8bit # stepper
payload_size = 13 & mask_8bit # 1 for command option, 4 for pos, 4 for speed, 4 for accel
first_byte = (type_select | payload_size) & mask_8bit
command_option = 3 & mask_8bit # move
pos = 100 & mask_32bit
speed = 100 & mask_32bit
accel = 100 & mask_32bit

packet = bytearray([
    first_byte,
    command_option,
    pos & 0xFF, (pos >> 8) & 0xFF, (pos >> 16) & 0xFF, (pos >> 24) & 0xFF,
    speed & 0xFF, (speed >> 8) & 0xFF, (speed >> 16) & 0xFF, (speed >> 24) & 0xFF, # little endian
    accel & 0xFF, (accel >> 8) & 0xFF, (accel >> 16) & 0xFF, (accel >> 24) & 0xFF
])

ser.write(packet)
print(packet)

resp = ser.readline()
print(resp)
