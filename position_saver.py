#!/usr/bin/env python
# -*- coding: utf-8 -*-

# *******************************************************************************
#
# Animatronic Position Capture Tool (Manifest & Address-Map based)
#
# Reads the current position of each servo from the manifest and saves
# the data to a new file in the "position" directory.
#
# *******************************************************************************

import os
import json
from dynamixel_sdk import *

# --- Constants ---
PROTOCOL_VERSION = 2.0
MANIFEST_FILENAME = os.path.join("config", "session_servos.json")
ADDRESS_FILENAME = os.path.join("config", "address.json")
POSITION_DIR = "position"


def load_json(filepath):
    """Loads a JSON file."""
    try:
        with open(filepath, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"[ERROR] Could not load or parse file '{filepath}': {e}")
        return None


def get_address_info(motor_type, address_map):
    """Gets the address block for a motor type, falling back to default."""
    return address_map.get(str(motor_type), address_map["default"])


def save_current_positions(queue, output_filepath):
    """Reads positions from manifest servos and saves to a file."""
    manifest = load_json(MANIFEST_FILENAME)
    address_map = load_json(ADDRESS_FILENAME)
    if not manifest or not address_map: return

    servos_by_port = {}
    for servo in manifest:
        port = servo['com_port']
        if port not in servos_by_port:
            servos_by_port[port] = {'baud_rate': servo['baud_rate'], 'servos': []}
        servos_by_port[port]['servos'].append(servo)

    packetHandler = PacketHandler(PROTOCOL_VERSION)
    found_servos_data = []

    for port_name, port_info in servos_by_port.items():
        portHandler = PortHandler(port_name)
        if not portHandler.openPort() or not portHandler.setBaudRate(port_info['baud_rate']):
            queue.put(f"[ERROR] Failed to connect to port {port_name}")
            continue

        queue.put(f"-> Reading servos on {port_name}...")
        for servo in port_info['servos']:
            dxl_id = servo['id']
            addr = get_address_info(servo['motor_type'], address_map)
            addr_pos = addr['present_position']
            len_pos = addr['len_present_position']

            # Dynamically choose read function based on data length
            read_func = {1: packetHandler.read1ByteTxRx, 2: packetHandler.read2ByteTxRx,
                         4: packetHandler.read4ByteTxRx}.get(len_pos)
            if not read_func:
                queue.put(f"  [WARN] Unsupported data length ({len_pos}) for ID {dxl_id}. Skipping.")
                continue

            pos, res, err = read_func(portHandler, dxl_id, addr_pos)
            if res == COMM_SUCCESS and err == 0:
                queue.put(f"  [SUCCESS] Read ID: {dxl_id}, Position: {pos}")
                found_servos_data.append({"ID": dxl_id, "position": pos})
            else:
                queue.put(f"  [WARN] Failed to read position for ID: {dxl_id}")
        portHandler.closePort()

    if not found_servos_data:
        queue.put("\nCould not read any servo positions.")
        return

    queue.put(f"\nSaving {len(found_servos_data)} positions...")
    try:
        sorted_data = sorted(found_servos_data, key=lambda x: x['ID'])
        with open(output_filepath, 'w') as f:
            json.dump(sorted_data, f, indent=4)
        queue.put(f"[SUCCESS] Position file saved to '{output_filepath}'")
    except Exception as e:
        queue.put(f"[ERROR] Failed to write file: {e}")