#!/usr/bin/env python3

import socket
import time
import threading
import re
import argparse
from typing import Dict

# --- Configuration ---
LISTEN_IP = "127.0.0.1"
COMMAND_PORT = 50001
FEEDBACK_PORT = 50002
DEBUG_PORT = 60000

NUM_AXES = 6
FEEDBACK_RATE_HZ = 100.0

# --- Global State ---
joint_angles_deg: Dict[int, float] = {i: 0.0 for i in range(1, NUM_AXES + 1)}
state_lock = threading.Lock()

# ==============================================================================
# --- EMULATOR LOGIC ---
# ==============================================================================

def parse_command(data: bytes) -> Dict[int, float]:
    """Parses a command string like 'A1:10.5,A2:-20.0' into a dictionary."""
    command_dict = {}
    try:
        command_str = data.decode('utf-8')
        pairs = command_str.strip().split(',')
        for pair in pairs:
            match = re.match(r"A(\d+):([-\d.]+)", pair)
            if match:
                axis = int(match.group(1))
                angle = float(match.group(2))
                if 1 <= axis <= NUM_AXES:
                    command_dict[axis] = angle
    except (UnicodeDecodeError, ValueError) as e:
        print(f"[Emulator:Command Parser] Error parsing command: {e}")
    return command_dict

def command_listener_thread():
    """Listens for incoming UDP command packets and updates the robot state."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((LISTEN_IP, COMMAND_PORT))
            print(f"[Emulator:Command Listener] Listening for commands on {LISTEN_IP}:{COMMAND_PORT}")

            while True:
                data, addr = sock.recvfrom(1024)
                # --- MODIFICATION: Log received command ---
                print(f"[Emulator] Received command: {data.decode()}")
                
                commands = parse_command(data)
                if commands:
                    with state_lock:
                        for axis, angle in commands.items():
                            joint_angles_deg[axis] = angle
        except OSError as e:
            print(f"[Emulator:Command Listener] ERROR: Could not bind to port {COMMAND_PORT}. Is another instance running? Details: {e}")
        except Exception as e:
            print(f"[Emulator:Command Listener] An error occurred: {e}")

def feedback_sender_thread():
    """Periodically sends the current robot state via UDP."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        print(f"[Emulator:Feedback Sender] Will send feedback to {LISTEN_IP}:{FEEDBACK_PORT} at {FEEDBACK_RATE_HZ} Hz")
        
        period = 1.0 / FEEDBACK_RATE_HZ
        while True:
            try:
                with state_lock:
                    current_angles = joint_angles_deg.copy()

                feedback_str = ",".join([f"A{i}:{current_angles[i]:.4f}" for i in range(1, NUM_AXES + 1)])
                sock.sendto(feedback_str.encode('utf-8'), (LISTEN_IP, FEEDBACK_PORT))

            except Exception as e:
                print(f"[Emulator:Feedback Sender] An error occurred: {e}")
            
            time.sleep(period)

# ==============================================================================
# --- DEBUG LISTENER LOGIC ---
# ==============================================================================

def debug_listener_thread():
    """Listens for and prints any incoming UDP debug packets."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((LISTEN_IP, DEBUG_PORT))
            print(f"[Debug Listener] Listening for debug packets on {LISTEN_IP}:{DEBUG_PORT}")

            while True:
                data, addr = sock.recvfrom(2048)
                print("\n--- DEBUG PACKET ---")
                print(f"From: {addr}")
                print(data.decode('utf-8'))
                print("--------------------\n")
        except OSError as e:
            print(f"[Debug Listener] ERROR: Could not bind to port {DEBUG_PORT}. Is another instance running? Details: {e}")
        except Exception as e:
            print(f"[Debug Listener] An unexpected error occurred: {e}")

# ==============================================================================
# --- MAIN EXECUTION ---
# ==============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Universal Robot Utility: Emulator and Debug Listener.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        '--emulator', 
        action='store_true', 
        help='Run the robot hardware emulator.\n'
             'Listens for commands on port 50001 and sends feedback to port 50002.'
    )
    parser.add_argument(
        '--debug-listener', 
        action='store_true', 
        help='Run the debug UDP packet listener on port 60000.'
    )
    args = parser.parse_args()

    threads = []

    if args.emulator:
        print("--- Starting Robot Hardware Emulator ---")
        threads.append(threading.Thread(target=command_listener_thread, daemon=True))
        threads.append(threading.Thread(target=feedback_sender_thread, daemon=True))
    
    if args.debug_listener:
        print("--- Starting Debug Packet Listener ---")
        threads.append(threading.Thread(target=debug_listener_thread, daemon=True))

    if not threads:
        print("No mode selected. Please specify --emulator or --debug-listener.")
        parser.print_help()
        exit(1)

    for t in threads:
        t.start()

    print("\nUtility running. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down.")