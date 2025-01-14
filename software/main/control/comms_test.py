from comms import Arm

arm1 = Arm(0)
arm1.stepper_home(10, 150, 10, 50, 10000)
while (True):
    arm1.stepper_move(100, 1000, 10000)
    arm1.block_until_stepper_move_complete()
    arm1.stepper_move(10, 1000, 10000)
    arm1.block_until_stepper_move_complete()
