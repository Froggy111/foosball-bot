#include "types.hpp"

// stepper driver
const types::u8 DRV_ENABLE = 8;
const types::u8 DRV_DIR = 1;
const types::u8 DRV_STEP = 2;
const types::u8 DRV_NSLEEP = 3;
const types::u8 DRV_MODE0 = 7;
const types::u8 DRV_MODE1 = 6;
const types::u8 DRV_MODE2 = 5;
const types::u8 DRV_NFAULT = 0;
const types::u8 DRV_NRESET = 4;

// endstop
// endstop is normally closed, and connected to ground.
// pullup on an input pin. it is low when triggered.
const types::u8 ENDSTOP = 9; 

// servo
const types::u8 SERVO_CTRL = 14;
const types::u8 SERVO_DIR = 15; // direction control of level shifter for servo. pull up to define rp2040 to servo direction.

// lidar
const types::u8 LIDAR_TX = 16;
const types::u8 LIDAR_RX = 17;
const types::u8 LIDAR_TX_DIR = 18;
const types::u8 LIDAR_RX_DIR = 19;
