from comms import Arm

arm1 = Arm(0)
print("created arm")
arm1.stepper_set_microsteps(4)
print("enable stepper")
arm1.stepper_enable()
print("set microsteps")
arm1.stepper_home(10, 150, 10, 50, 10000)
print("homed stepper")
while (True):
    print("moving stepper forward")
    arm1.stepper_move(100, 1000, 10000)
    arm1.move_servo(0)
    arm1.block_until_stepper_move_complete()
    print("moving stepper backward")
    arm1.stepper_move(10, 1000, 10000)
    arm1.move_servo(180*4)
    arm1.block_until_stepper_move_complete()
