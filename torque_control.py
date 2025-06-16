#!/usr/bin/env python
# -*- coding: utf-8 -*-

# *******************************************************************************
#
# A utility script to enable or disable torque on all servos defined in the
# manifest, using a dynamic address map from "config/address.json".
#
# *******************************************************************************

import os
import sys
import json
from dynamixel_sdk import *

# --- Constants ---
PROTOCOL_VERSION = 2.0
MANIFEST_FILENAME = os.path.join("config", "session_servos.json")
ADDRESS_FILENAME = os.path.join("config", "address.json")
TORQUE_ENABLE_VAL = 1
TORQUE_DISABLE_VAL = 0


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


def set_torque(queue, state):
    """Sets torque on or off for all servos in the manifest."""
    if state not in ['on', 'off']:
        queue.put(f"[ERROR] Invalid torque state: {state}")
        return

    torque_value = TORQUE_ENABLE_VAL if state == 'on' else TORQUE_DISABLE_VAL
    action_string = "Enabling" if state == 'on' else "Disabling"

    queue.put(f"--- {action_string} torque for all servos ---")

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

    for port_name, port_data in servos_by_port.items():
        portHandler = PortHandler(port_name)
        if portHandler.openPort() and portHandler.setBaudRate(port_data['baud_rate']):
            queue.put(f"-> Sending command to port {port_name}")
            # Use GroupBulkWrite for flexibility with different motor types
            groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

            for servo in port_data['servos']:
                addr = get_address_info(servo['motor_type'], address_map)
                addr_torque = addr['torque_enable']
                len_torque = addr['len_torque_enable']

                # Prepare data based on its length
                param_torque = [DXL_LOBYTE(torque_value), DXL_HIBYTE(torque_value)][:len_torque]
                groupBulkWrite.addParam(servo['id'], addr_torque, len_torque, param_torque)

            if groupBulkWrite.txPacket() == COMM_SUCCESS:
                queue.put(f"  [SUCCESS] {action_string} command sent.")
            else:
                queue.put(f"  [FAILURE] Failed to send command on port {port_name}")
            portHandler.closePort()
        else:
            queue.put(f"[ERROR] Failed to open port {port_name}")

    queue.put(f"--- Torque {state.upper()} command finished ---")