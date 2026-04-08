import socket
import time
import json
import os

def read_ip_from_txt(filename):
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            return file.readline().strip()
    else:
        print(f"Warning: File '{filename}' not found. Using default IP.")
        return "127.0.0.1"

# --- 1. FORWARD KINEMATICS (Wheels -> Linear/Angular) ---
def wheels_to_twist(fl, fr, bl, br, track_width):
    # Average the commands for the left and right sides
    cmd_left = (fl + bl) / 2.0
    cmd_right = (fr + br) / 2.0
    
    # Linear velocity (v): Negative commands mean positive forward movement
    v = -(cmd_right + cmd_left) / 2.0
    
    # Angular velocity (w): Left turns (positive w) require right wheels to go forward (negative cmd)
    # and left wheels to go backward (positive cmd).
    w = (cmd_left - cmd_right) / track_width
    
    return v, w

# --- 2. INVERSE KINEMATICS (Linear/Angular -> Wheels) ---
def twist_to_wheels(v, w, track_width):
    # Calculate actual physical velocity needed for each side
    vel_left = v - (w * track_width / 2.0)
    vel_right = v + (w * track_width / 2.0)
    
    # Invert the physical velocity to get the motor command (negative means forward)
    cmd_left = -vel_left
    cmd_right = -vel_right
    
    return {
        "FL": cmd_left,
        "BL": cmd_left,
        "FR": cmd_right,
        "BR": cmd_right
    }

# Network Setup
windows_ip = read_ip_from_txt("ip.txt")
port = 3002
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(f"Sending UDP packets to: {windows_ip}:{port}")

# --- ROVER PHYSICAL PARAMETER ---
# Replace this with the actual distance between the left and right wheels in your simulator!
TRACK_WIDTH = 1.0 # (In meters)

while True:
    # ---------------------------------------------------------
    # EXAMPLE A: You have wheel commands and want to know V and W
    # ---------------------------------------------------------
    current_commands = {"BL": -0.5, "BR": -0.51, "FL": -0.5, "FR": -0.51}
    
    v_actual, w_actual = wheels_to_twist(
        current_commands["FL"], current_commands["FR"], 
        current_commands["BL"], current_commands["BR"], 
        TRACK_WIDTH
    )
    print(f"Reading -> Linear: {v_actual:.3f} m/s | Angular: {w_actual:.3f} rad/s")

    # ---------------------------------------------------------
    # EXAMPLE B: You want to CONTROL the rover using V and W
    # ---------------------------------------------------------
    # Let's say you want to move forward at 0.5 m/s and turn slightly left at 0.1 rad/s
    target_v = 0.5  # Positive = Forward
    target_w = 0.5  # Positive = Turn Left (Counter-Clockwise)
    
    # The script automatically calculates the inverted FL, FR, BL, BR commands
    data_to_send = twist_to_wheels(target_v, target_w, TRACK_WIDTH)
    
    # Send via UDP
    json_string = json.dumps(data_to_send)
    s.sendto(json_string.encode('utf-8'), (windows_ip, port))
    
    time.sleep(0.1)