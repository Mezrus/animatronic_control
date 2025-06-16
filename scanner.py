import os
import json
import serial.tools.list_ports
from dynamixel_sdk import *

# Constants
PROTOCOL_VERSION = 2.0
DXL_ID_RANGE = range(0, 101)
BAUDRATES = [57600, 115200, 1000000, 2000000, 4000000]
OUTPUT_DIR = "config"
OUTPUT_FILENAME = os.path.join(OUTPUT_DIR, "session_servos.json")


def get_available_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


def run_scan(queue, success_message):
    """Main function to scan all ports and save a servo manifest file."""
    queue.put("============================================")
    queue.put("  Running Initial System Scan...")
    queue.put("============================================")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    available_ports = get_available_ports()

    if not available_ports:
        queue.put("[ERROR] No COM ports found.")
        return

    all_found_servos = []
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    for port_name in available_ports:
        portHandler = PortHandler(port_name)
        if not portHandler.openPort():
            continue

        for baudrate in BAUDRATES:
            if not portHandler.setBaudRate(baudrate):
                continue

            queue.put(f"-> Scanning {port_name} @ {baudrate} bps...")
            for dxl_id in DXL_ID_RANGE:
                dxl_model, dxl_comm, dxl_error = packetHandler.ping(portHandler, dxl_id)
                if dxl_comm == COMM_SUCCESS and dxl_error == 0:
                    servo_info = {"com_port": port_name, "baud_rate": baudrate, "id": dxl_id, "motor_type": dxl_model}
                    queue.put(f"  [SUCCESS] Found Servo ID: {dxl_id}")
                    all_found_servos.append(servo_info)
        portHandler.closePort()

    if not all_found_servos:
        queue.put("\nScan Complete. No servos found.")
        return

    queue.put(f"\nScan Complete. Found {len(all_found_servos)} servos.")
    queue.put(f"Saving manifest to '{OUTPUT_FILENAME}'...")
    try:
        sorted_servos = sorted(all_found_servos, key=lambda x: x['id'])
        with open(OUTPUT_FILENAME, 'w') as f:
            json.dump(sorted_servos, f, indent=4)
        queue.put(f"[SUCCESS] Manifest saved.")
    except Exception as e:
        queue.put(f"[ERROR] Failed to save manifest: {e}")

    queue.put(success_message)