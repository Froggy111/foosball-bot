from comms import Arm

arm0 = Arm(0)
arm1 = Arm(1)
arm2 = Arm(2)
print("created arm")
arm0.stepper_set_microsteps(4)
arm1.stepper_set_microsteps(4)
arm2.stepper_set_microsteps(4)
print("enable stepper")
arm0.stepper_enable()
arm1.stepper_enable()
arm2.stepper_enable()
print("set microsteps")
arm0.stepper_home(10, 150, 10, 50, 10000)
arm1.stepper_home(10, 150, 10, 50, 10000)
arm2.stepper_home(10, 150, 10, 50, 10000)
print("homed stepper")
while (True):
    print("moving stepper forward")
    arm0.stepper_move(100, 1000, 10000)
    arm1.stepper_move(100, 1000, 10000)
    arm2.stepper_move(100, 1000, 10000)
    arm0.move_servo(0)
    arm1.move_servo(0)
    arm2.move_servo(0)
    arm0.block_until_stepper_move_complete()
    arm1.block_until_stepper_move_complete()
    arm2.block_until_stepper_move_complete()
    print("moving stepper backward")
    arm0.stepper_move(10, 1000, 10000)
    arm1.stepper_move(10, 1000, 10000)
    arm2.stepper_move(10, 1000, 10000)
    arm0.move_servo(180*4)
    arm1.move_servo(180*4)
    arm2.move_servo(180*4)
    arm0.block_until_stepper_move_complete()
    arm1.block_until_stepper_move_complete()
    arm2.block_until_stepper_move_complete()
