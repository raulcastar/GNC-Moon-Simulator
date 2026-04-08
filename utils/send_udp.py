import socket
import time
import json
import os

def read_ip_from_txt(filename):
    # Check if the file exists first
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            # Read the first line and use .strip() to remove invisible spaces or line breaks
            ip = file.readline().strip()
            return ip
    else:
        print(f"Warning: File '{filename}' not found. Using default IP.")
        return "127.0.0.1"

# Read the IP from the file
windows_ip = read_ip_from_txt("ip.txt")
port = 3002

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Sending UDP packets to the IP read from file: {windows_ip}:{port}")

while True:
    data = {
        "BL": -0.5,
        "BR": -0.51,
        "FL": -0.5,
        "FR": -0.51
    }
    
    # Convert to JSON and send
    json_string = json.dumps(data)
    s.sendto(json_string.encode('utf-8'), (windows_ip, port))
    
    time.sleep(0.1)