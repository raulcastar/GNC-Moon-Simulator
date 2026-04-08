import socket
import json

port = 3001
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 0.0.0.0 tells WSL to listen for anything crossing the bridge from Windows
s.bind(("0.0.0.0", port))

print(f"Listening for Unreal Engine data on port {port}...")

while True:
    data, addr = s.recvfrom(1024)
    message = data.decode('utf-8')
    
    try:
        transform = json.loads(message)
        timestamp_ms = transform['time']
        
        print(f"[{timestamp_ms} ms] Location -> X: {transform['x']}, Y: {transform['y']}, Z: {transform['z']}")
        print(f"           Rotation -> Roll: {transform['roll']}, Pitch: {transform['pitch']}, Yaw: {transform['yaw']}")
    except json.JSONDecodeError:
        print(f"Received non-JSON data: {message}")