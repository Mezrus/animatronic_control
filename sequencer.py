#!/usr/bin/env python
# -*- coding: utf-8 -*-

# *******************************************************************************
#
# Animatronic Sequencer Module (Address-Aware)
#
# This module reads all control table addresses from "config/address.json"
# and uses GroupBulkRead/Write to handle potentially mixed servo models.
#
# *******************************************************************************

import os
import sys
import json
import time

try:
    import toml
except ImportError:
    sys.exit("[ERROR] The 'toml' library is not installed. Please run: pip install toml")
from dynamixel_sdk import *

# --- Constants ---
PROTOCOL_VERSION = 2.0
MANIFEST_FILENAME = os.path.join("config", "session_servos.json")
ADDRESS_FILENAME = os.path.join("config", "address.json")
DEFAULT_BASE_VELOCITY = 150


# --- Helper Functions ---
def get_address_info(motor_type, address_map):
    """Gets the address block for a motor type, falling back to default."""
    return address_map.get(str(motor_type), address_map["default"])


def check_torque_status(queue, servo_manifest, port_handlers, packetHandler, address_map):
    queue.put("Performing pre-flight torque check...")
    for port_name, port_handler in port_handlers.items():
        groupBulkRead = GroupBulkRead(port_handler, packetHandler)
        servos_on_port = [s for s_id, s in servo_manifest.items() if s['com_port'] == port_name]

        for servo in servos_on_port:
            addr = get_address_info(servo['motor_type'], address_map)
            groupBulkRead.addParam(servo['id'], addr['torque_enable'], addr['len_torque_enable'])

        if groupBulkRead.txRxPacket() != COMM_SUCCESS: return False

        for servo in servos_on_port:
            addr = get_address_info(servo['motor_type'], address_map)
            if not groupBulkRead.isAvailable(servo['id'], addr['torque_enable'], addr['len_torque_enable']) or \
                    groupBulkRead.getData(servo['id'], addr['torque_enable'], addr['len_torque_enable']) != 1:
                queue.put(f"[CRITICAL] Torque is disabled on servo ID {servo['id']}.")
                return False
    queue.put("[SUCCESS] Torque check passed.")
    return True


def execute_sync_move(queue, servo_goals, base_velocity, servo_manifest, port_handlers, packetHandler, address_map):
    moves_by_port = {}
    for dxl_id, goal_pos in servo_goals.items():
        if dxl_id not in servo_manifest: continue
        port_name = servo_manifest[dxl_id]['com_port']
        if port_name not in moves_by_port: moves_by_port[port_name] = {'goals': {}}
        moves_by_port[port_name]['goals'][dxl_id] = goal_pos

    for port_name, move_data in moves_by_port.items():
        if port_name not in port_handlers: continue
        port_handler = port_handlers[port_name]

        # Read present positions using GroupBulkRead
        groupBulkRead_Pos = GroupBulkRead(port_handler, packetHandler)
        for dxl_id in move_data['goals']:
            addr = get_address_info(servo_manifest[dxl_id]['motor_type'], address_map)
            groupBulkRead_Pos.addParam(dxl_id, addr['present_position'], addr['len_present_position'])
        if groupBulkRead_Pos.txRxPacket() != COMM_SUCCESS: continue

        # Prepare bulk writes for velocity and position
        groupBulkWrite_Vel = GroupBulkWrite(port_handler, packetHandler)
        groupBulkWrite_Pos = GroupBulkWrite(port_handler, packetHandler)

        max_travel, servo_travel_data = 0, {}
        for dxl_id in move_data['goals']:
            addr = get_address_info(servo_manifest[dxl_id]['motor_type'], address_map)
            present_pos = groupBulkRead_Pos.getData(dxl_id, addr['present_position'], addr['len_present_position'])
            travel = abs(move_data['goals'][dxl_id] - present_pos)
            servo_travel_data[dxl_id] = travel
            if travel > max_travel: max_travel = travel

        if max_travel > 0:
            for dxl_id, goal_pos in move_data['goals'].items():
                addr = get_address_info(servo_manifest[dxl_id]['motor_type'], address_map)
                velocity = int(max(1, (servo_travel_data[dxl_id] / max_travel) * base_velocity))

                param_vel = [DXL_LOBYTE(DXL_LOWORD(velocity)), DXL_HIBYTE(DXL_LOWORD(velocity)),
                             DXL_LOBYTE(DXL_HIWORD(velocity)), DXL_HIBYTE(DXL_HIWORD(velocity))][
                            :addr['len_profile_velocity']]
                groupBulkWrite_Vel.addParam(dxl_id, addr['profile_velocity'], addr['len_profile_velocity'], param_vel)

                param_goal = [DXL_LOBYTE(DXL_LOWORD(goal_pos)), DXL_HIBYTE(DXL_LOWORD(goal_pos)),
                              DXL_LOBYTE(DXL_HIWORD(goal_pos)), DXL_HIBYTE(DXL_HIWORD(goal_pos))][
                             :addr['len_goal_position']]
                groupBulkWrite_Pos.addParam(dxl_id, addr['goal_position'], addr['len_goal_position'], param_goal)

            groupBulkWrite_Vel.txPacket()
            groupBulkWrite_Pos.txPacket()
    return True


def wait_for_move_completion(queue, servo_goals, servo_manifest, port_handlers, packetHandler, address_map):
    servos_to_check = set(servo_goals.keys())
    servos_by_port = {}
    for dxl_id in servos_to_check:
        if dxl_id not in servo_manifest: continue
        port_name = servo_manifest[dxl_id]['com_port']
        if port_name not in servos_by_port: servos_by_port[port_name] = []
        servos_by_port[port_name].append(dxl_id)

    queue.put("Waiting for movement to complete...")
    while servos_to_check:
        for port_name, all_ids_on_port in servos_by_port.items():
            ids_to_poll = list(servos_to_check.intersection(all_ids_on_port))
            if not ids_to_poll: continue
            if port_name not in port_handlers: continue

            port_handler = port_handlers[port_name]
            groupBulkRead = GroupBulkRead(port_handler, packetHandler)
            for dxl_id in ids_to_poll:
                addr = get_address_info(servo_manifest[dxl_id]['motor_type'], address_map)
                groupBulkRead.addParam(dxl_id, addr['moving'], addr['len_moving'])

            if groupBulkRead.txRxPacket() != COMM_SUCCESS: continue

            for dxl_id in ids_to_poll:
                addr = get_address_info(servo_manifest[dxl_id]['motor_type'], address_map)
                if groupBulkRead.isAvailable(dxl_id, addr['moving'], addr['len_moving']) and \
                        groupBulkRead.getData(dxl_id, addr['moving'], addr['len_moving']) == 0:
                    servos_to_check.remove(dxl_id)

        if servos_to_check: time.sleep(0.02)
    queue.put("Movement complete.")


def process_sequence(queue, filepath, speed_multiplier, servo_manifest, port_handlers, packetHandler, address_map):
    # This function's internal logic remains largely the same, but it passes address_map down
    try:
        sequence_data = toml.load(filepath)
    except Exception as e:
        queue.put(f"[ERROR] Could not load TOML file '{filepath}': {e}");
        return

    base_velocity = sequence_data.get('base_velocity', DEFAULT_BASE_VELOCITY)
    steps = sequence_data.get('sequence', [])
    queue.put(f"--- Processing Sequence: {os.path.basename(filepath)} at {speed_multiplier * 100:.0f}% speed ---")

    for i, step in enumerate(steps):
        position_file = step.get('position_file')
        if not position_file: continue
        queue.put(f"-- Step {i + 1}/{len(steps)}: '{position_file}' --")

        if position_file.lower().endswith('.toml'):
            nested_speed = step.get('speed', 100) / 100.0
            process_sequence(queue, os.path.join("animation", position_file), speed_multiplier * nested_speed,
                             servo_manifest, port_handlers, packetHandler, address_map)
        elif position_file.lower().endswith('.json'):
            json_filepath = os.path.join("position", position_file)
            try:
                with open(json_filepath, 'r') as f:
                    step_goals = {item['ID']: item['position'] for item in json.load(f)}
                step_velocity = step.get('velocity', base_velocity)
                final_velocity = int(step_velocity * speed_multiplier)
                if execute_sync_move(queue, step_goals, final_velocity, servo_manifest, port_handlers, packetHandler,
                                     address_map):
                    wait_for_move_completion(queue, step_goals, servo_manifest, port_handlers, packetHandler,
                                             address_map)
            except Exception as e:
                queue.put(f"[ERROR] Could not process JSON file '{json_filepath}': {e}")

        delay = step.get('delay_ms', 0) / 1000.0
        if delay > 0:
            queue.put(f"Waiting for {delay:.2f} seconds...")
            time.sleep(delay)
    queue.put(f"--- Sequence '{os.path.basename(filepath)}' Complete ---")


def run_animation(queue, filename):
    """Main entry point for the GUI to run a full animation."""
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    port_handlers = {}

    try:
        with open(MANIFEST_FILENAME, 'r') as f:
            servo_manifest = {item['id']: item for item in json.load(f)}
        with open(ADDRESS_FILENAME, 'r') as f:
            address_map = json.load(f)
    except Exception as e:
        queue.put(f"[FATAL ERROR] Cannot load manifest or address file: {e}");
        return

    unique_ports = {s['com_port']: s['baud_rate'] for s in servo_manifest.values()}
    for port_name, baud_rate in unique_ports.items():
        try:
            port_handler = PortHandler(port_name)
            if not port_handler.openPort() or not port_handler.setBaudRate(baud_rate): raise ConnectionError()
            port_handlers[port_name] = port_handler
        except Exception:
            queue.put(f"[ERROR] Failed to open connection to '{port_name}'.")

    if len(port_handlers) != len(unique_ports):
        for handler in port_handlers.values(): handler.closePort()
        return

    if not check_torque_status(queue, servo_manifest, port_handlers, packetHandler, address_map):
        for handler in port_handlers.values(): handler.closePort()
        queue.put("--- Animation aborted. ---");
        return

    if filename.lower().endswith('.toml'):
        process_sequence(queue, os.path.join("animation", filename), 1.0, servo_manifest, port_handlers, packetHandler,
                         address_map)
    elif filename.lower().endswith('.json'):
        try:
            with open(os.path.join("position", filename), 'r') as f:
                servo_goals = {item['ID']: item['position'] for item in json.load(f)}
            if execute_sync_move(queue, servo_goals, DEFAULT_BASE_VELOCITY, servo_manifest, port_handlers,
                                 packetHandler, address_map):
                wait_for_move_completion(queue, servo_goals, servo_manifest, port_handlers, packetHandler, address_map)
        except Exception as e:
            queue.put(f"[ERROR] Could not process JSON file: {e}")

    for port_handler in port_handlers.values(): port_handler.closePort()
    queue.put(f"--- Finished '{filename}' ---")