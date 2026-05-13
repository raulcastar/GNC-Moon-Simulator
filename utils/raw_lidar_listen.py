import socket

# Configuration - Match this to your Unreal 'ScanPort'
IP = "127.0.0.1"
PORT = 2368 

# Create the socket
# AF_INET = IPv4, SOCK_DGRAM = UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Re-use the address so you don't get 'Port in use' errors if you restart quickly
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

try:
    sock.bind((IP, PORT))
    print(f"--- Listening for RAW UDP on {IP}:{PORT} ---")
    print("Hit Play in Unreal Engine now...")
except Exception as e:
    print(f"ERROR: Could not bind to port {PORT}. Is another script running? {e}")
    exit()

count = 0
try:
    while True:
        # 1206 is the standard Velodyne packet size
        data, addr = sock.recvfrom(2048)
        count += 1
        
        # Format the first 20 bytes as Hex for debugging
        hex_data = ' '.join(f'{b:02x}' for b in data[:20])
        
        print(f"Packet #{count} | From: {addr}")
        print(f"  Size: {len(data)} bytes")
        print(f"  Raw Hex (first 20b): {hex_data}")
        print("-" * 50)

except KeyboardInterrupt:
    print("\nStopping listener...")
finally:
    sock.close()
