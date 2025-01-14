import serial
from serial import Serial
import serial.tools.list_ports
from enum import IntEnum

mask8 = 0xFF
mask16 = 0xFFFF
mask32 = 0xFFFFFFFF
def to_bytelist_16bit(a, signed: bool = False):
    return list(int.to_bytes(a & mask16, 2, "little", signed=signed))
def to_bytelist_32bit(a, signed: bool = False):
    return list(int.to_bytes(a & mask32, 4, "little", signed=signed))
def from_bytelist(a, signed: bool = False):
    return int.from_bytes(bytes(a), "little", signed=signed)

class TargetCore(IntEnum):
    core0 = 0b00000000
    core1 = 0b10000000

class Core0CommandIDs(IntEnum):
    estop = 0 & mask8
    restart = 1 & mask8
    
    move_servo = 2 & mask8

class Core0PayloadLengths(IntEnum):
    estop = 1 & mask8
    restart = 1 & mask8

    move_servo = 3 & mask8

class Core1CommandIDs(IntEnum):
    enable = 0 & mask8
    disable = 1 & mask8
    reset = 2 & mask8
    move = 3 & mask8
    home = 4 & mask8

    fault_state = 5 & mask8
    get_max_speed = 6 & mask8
    get_max_accel = 7 & mask8
    get_steps_per_rev = 8 & mask8
    get_mm_per_rev = 9 & mask8
    get_um_per_step = 10 & mask8
    get_max_steps_per_second = 11 & mask8
    get_max_steps_accel = 12 & mask8
    get_microsteps = 13 & mask8

    set_max_speed = 14 & mask8
    set_max_accel = 15 & mask8
    set_steps_per_rev = 16 & mask8
    set_mm_per_rev = 17 & mask8
    set_microsteps = 18 & mask8

    get_step_coord = 19 & mask8
    get_um_coord = 20 & mask8
    get_current_speed = 21 & mask8
    set_step_coord = 22 & mask8

class Core1PayloadLengths(IntEnum):
    enable = 1 & mask8
    disable = 1 & mask8
    reset = 1 & mask8
    move = 13 & mask8
    home = 12 & mask8

    fault_state = 1 & mask8
    get_max_speed = 1 & mask8
    get_max_accel = 1 & mask8
    get_steps_per_rev = 1 & mask8
    get_mm_per_rev = 1 & mask8
    get_um_per_step = 1 & mask8
    get_max_steps_per_second = 1 & mask8
    get_max_steps_accel = 1 & mask8
    get_microsteps = 1 & mask8

    set_max_speed = 5 & mask8
    set_max_accel = 5 & mask8
    set_steps_per_rev = 3 & mask8
    set_mm_per_rev = 2 & mask8
    set_microsteps = 2 & mask8

    get_step_coord = 1 & mask8
    get_um_coord = 1 & mask8
    get_current_speed = 1 & mask8
    set_step_coord = 5 & mask8

stepper_move_complete_ack_val = 0b01010101


class Arm:
    def __init__(self, id: int) -> None:
        self._setup_serial(id)
        self._sent_command = False
    def _setup_serial(self, id: int) -> None:
        vid = "2E8A" # usb vid
        pid = f"FFF{id+1}" # usb pid
        pattern = f"(?i)VID:PID={vid}:{pid}"
        ports_iter = serial.tools.list_ports.grep(pattern)
        self.serial_port_name, _, _ = list(ports_iter)[0]
        self.serial_port = Serial(self.serial_port_name)
        self.serial_port.set_low_latency_mode(True)
    def _send_cmd(self, cmd: bytearray) -> None:
        self.serial_port.write(cmd)
        print(cmd)
    def _block_until_complete_ack(self) -> None:
        complete_ack = self.serial_port.read(1)
        assert complete_ack, "complete_ack byte is false."

    # actions
    def estop(self) -> None:
        payload = bytearray([
            TargetCore.core0 & Core0PayloadLengths.estop, # first byte
            Core0CommandIDs.estop # command id byte
        ])
        self._send_cmd(payload)
    def restart(self) -> None:
        payload = bytearray([
            TargetCore.core0 & Core0PayloadLengths.restart, # first byte
            Core0CommandIDs.restart # command id byte
        ])
        self._send_cmd(payload)
    def move_servo(self, angle: int) -> None:
        payload = bytearray([
            TargetCore.core0 & Core0PayloadLengths.move_servo, # first byte
            Core0CommandIDs.move_servo # command id byte
        ] + to_bytelist_16bit(angle)) # data bytes
        self._send_cmd(payload)

    def stepper_enable(self) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.enable, # first byte
            Core1CommandIDs.enable # command id byte
        ])
        self._send_cmd(payload)
        self._block_until_complete_ack()
    def stepper_disable(self) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.disable, # first byte
            Core1CommandIDs.disable # command id byte
        ])
        self._send_cmd(payload)
        self._block_until_complete_ack()
    def stepper_reset(self) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.reset, # first byte
            Core1CommandIDs.reset # command id byte
        ])
        self._send_cmd(payload)
        self._block_until_complete_ack()
    def stepper_move(self, move_pos: int, speed: int, accel: int) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.move, # first byte
            Core1CommandIDs.move # command id byte
        ] + to_bytelist_32bit(move_pos, True) # data bytes
          + to_bytelist_32bit(speed)
          + to_bytelist_32bit(accel))
        self._send_cmd(payload)
        self._block_until_complete_ack()
    def block_until_stepper_move_complete(self) -> None:
        stepper_move_complete_ack = self.serial_port.read(1)
        assert stepper_move_complete_ack == stepper_move_complete_ack_val, "Stepper move complete ack value mismatch."
    def stepper_home(self, moveforward_mm: int, movebackward_mm: int, target_pos: int, speed: int, accel: int) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.home, # first byte
            Core1CommandIDs.home, # command id byte
            moveforward_mm & mask8,
            movebackward_mm & mask8,
            target_pos & mask8
        ]
          + to_bytelist_32bit(speed)
          + to_bytelist_32bit(accel))
        self._send_cmd(payload)
        self._block_until_complete_ack()
    
    # config reads
    def stepper_fault_state(self) -> bool:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.fault_state, # first byte
            Core1CommandIDs.fault_state # command id byte
        ])
        self._send_cmd(payload)
        return bool(self.serial_port.read(1))
    def stepper_get_max_speed(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_max_speed, # first byte
            Core1CommandIDs.get_max_speed # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(4))
    def stepper_get_max_accel(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_max_accel, # first byte
            Core1CommandIDs.get_max_accel # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(4))
    def stepper_get_steps_per_rev(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_steps_per_rev, # first byte
            Core1CommandIDs.get_steps_per_rev # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(2))
    def stepper_get_mm_per_rev(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_mm_per_rev, # first byte
            Core1CommandIDs.get_mm_per_rev # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(1))
    def stepper_get_um_per_step(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_um_per_step, # first byte
            Core1CommandIDs.get_um_per_step # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(2))
    def stepper_get_max_steps_per_second(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_max_steps_per_second, # first byte
            Core1CommandIDs.get_max_steps_per_second # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(4))
    def stepper_get_max_steps_accel(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_max_steps_accel, # first byte
            Core1CommandIDs.get_max_steps_accel # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(4))
    def stepper_get_microsteps(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_microsteps, # first byte
            Core1CommandIDs.get_microsteps # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(1))

    # config writes
    def stepper_set_max_speed(self, max_speed: int) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.set_max_speed, # first byte
            Core1CommandIDs.set_max_speed # command id byte
        ] + to_bytelist_32bit(max_speed))
        self._send_cmd(payload)
        self._block_until_complete_ack()
    def stepper_set_max_accel(self, max_accel: int) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.set_max_accel, # first byte
            Core1CommandIDs.set_max_accel # command id byte
        ] + to_bytelist_32bit(max_accel))
        self._send_cmd(payload)
        self._block_until_complete_ack()
    def stepper_set_steps_per_rev(self, steps_per_rev: int) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.set_steps_per_rev, # first byte
            Core1CommandIDs.set_steps_per_rev # command id byte
        ] + to_bytelist_16bit(steps_per_rev))
        self._send_cmd(payload)
        self._block_until_complete_ack()
    def stepper_set_mm_per_rev(self, mm_per_rev: int) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.set_mm_per_rev, # first byte
            Core1CommandIDs.set_mm_per_rev, # command id byte
            mm_per_rev & mask8
        ])
        self._send_cmd(payload)
        self._block_until_complete_ack()
    def stepper_set_microsteps(self, microsteps: int) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.set_microsteps, # first byte
            Core1CommandIDs.set_microsteps, # command id byte
            microsteps & mask8
        ])
        self._send_cmd(payload)
        self._block_until_complete_ack()

    # state machine
    def stepper_get_step_coord(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_step_coord, # first byte
            Core1CommandIDs.get_step_coord # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(4), True)
    def stepper_get_um_coord(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_um_coord, # first byte
            Core1CommandIDs.get_um_coord # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(4), True)
    def stepper_get_current_speed(self) -> int:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.get_current_speed, # first byte
            Core1CommandIDs.get_current_speed # command id byte
        ])
        self._send_cmd(payload)
        return from_bytelist(self.serial_port.read(4), True)
    def stepper_set_step_coord(self, step_coord: int) -> None:
        payload = bytearray([
            TargetCore.core1 & Core1PayloadLengths.set_step_coord, # first byte
            Core1CommandIDs.set_step_coord, # command id byte
        ] + to_bytelist_32bit(step_coord))
        self._send_cmd(payload)
        self._block_until_complete_ack()
