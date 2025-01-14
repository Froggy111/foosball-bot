import serial
import serial.tools.list_ports

def get_serial_port(id: int) -> tuple[str, str, str]:
    vid = "2E8A" # usb vid
    pid = f"FFF{id+1}" # usb pid
    pattern = f"(?i)VID:PID={vid}:{pid}"
    ports_iter = serial.tools.list_ports.grep(pattern)
    port = next(ports_iter)
    return port

def setup_serial_port(port_name: str) -> serial.Serial:
    
