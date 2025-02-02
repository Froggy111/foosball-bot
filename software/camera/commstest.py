import serial
import time as t

ser = serial.Serial('/dev/ttyACM0',115200)
ser.set_low_latency_mode(True)

mask_8bit = 0xFF
mask_16bit = 0xFFFF
mask_32bit = 0xFFFFFFFF

type_select = 0b10000000 & mask_8bit # stepper
payload_size = 13 & mask_8bit # 1 for command option, 4 for pos, 4 for speed, 4 for accel
first_byte = (type_select | payload_size) & mask_8bit
command_option = 3 & mask_8bit # move
pos = 100 & mask_32bit
speed = 1000 & mask_32bit
accel = 10000 & mask_32bit
pos2 = 0 & mask_32bit

microstep_payload_size = 2 & mask_8bit
microstep_first_byte = (type_select | microstep_payload_size) & mask_8bit
microstep_cmd_option = 18 & mask_8bit
microsteps = 4 & mask_8bit

homing_payload_size = 12 & mask_8bit
homing_first_byte = (type_select | homing_payload_size) & mask_8bit
homing_cmd_option = 4 & mask_8bit
homing_moveforward_mm = 10 & mask_8bit
homing_movebackward_mm = 150 & mask_8bit
homing_target_pos = 10 & mask_8bit
homing_speed = 50 & mask_32bit
homing_accel = 10000 & mask_32bit

microstep_packet = bytearray([
    microstep_first_byte,
    microstep_cmd_option,
    microsteps
])

homing_packet = bytearray([
    homing_first_byte,
    homing_cmd_option,
    homing_moveforward_mm,
    homing_movebackward_mm,
    homing_target_pos,
    homing_speed & 0xFF, (homing_speed >> 8) & 0xFF, (homing_speed >> 16) & 0xFF, (homing_speed >> 24) & 0xFF, # little endian
    homing_accel & 0xFF, (homing_accel >> 8) & 0xFF, (homing_accel >> 16) & 0xFF, (homing_accel >> 24) & 0xFF
])

packet = bytearray([
    first_byte,
    command_option,
    pos & 0xFF, (pos >> 8) & 0xFF, (pos >> 16) & 0xFF, (pos >> 24) & 0xFF,
    speed & 0xFF, (speed >> 8) & 0xFF, (speed >> 16) & 0xFF, (speed >> 24) & 0xFF, # little endian
    accel & 0xFF, (accel >> 8) & 0xFF, (accel >> 16) & 0xFF, (accel >> 24) & 0xFF
])

packet2 = bytearray([
    first_byte,
    command_option,
    pos2 & 0xFF, (pos2 >> 8) & 0xFF, (pos2 >> 16) & 0xFF, (pos2 >> 24) & 0xFF,
    speed & 0xFF, (speed >> 8) & 0xFF, (speed >> 16) & 0xFF, (speed >> 24) & 0xFF, # little endian
    accel & 0xFF, (accel >> 8) & 0xFF, (accel >> 16) & 0xFF, (accel >> 24) & 0xFF
])

ser.write(microstep_packet)
print(microstep_packet)
print(ser.read(1))

ser.write(homing_packet)
print(homing_packet)
print(ser.read(1))

ser.write(packet)
print(packet)
print(ser.read(2))

# resp = ser.readline().decode()
# print(resp)
# resp = ser.readline().decode()
# print(resp)
# resp = ser.readline().decode()
# print(resp)
# tsleep = 0.0001
# tstart = t.perf_counter()
# while (t.perf_counter() - tstart < tsleep):
#     continue

ser.write(packet2)
print(packet2)
print(ser.read(2))
# resp = ser.readline().decode()
# print(resp)
# resp = ser.readline().decode()
# print(resp)
# resp = ser.readline().decode()
# print(resp)
