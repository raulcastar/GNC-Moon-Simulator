from unrealcv import Client
import time
import os
import numpy as np
from unrealcv.util import read_png, read_npy
import matplotlib
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt

# 1. Read IP from file
def read_ip_from_txt(filename):
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            return file.readline().strip()
    return "127.0.0.1"

HOST_IP = read_ip_from_txt("ip.txt")
client = Client((HOST_IP, 9000), 'inet')

def check_connection(client):
    while client.isconnected() is False:
        print('Trying to connect to UnrealCV...')
        client.connect()
        time.sleep(1)
    print('Connection Established!')

check_connection(client)

# --- CAPTURE (Real-time, Game Unpaused) ---
try:
    print("Capturing Triple Camera Data (Real-time mode)...")
    
    # Camera 1: Left RGB
    res_l_rgb = client.request('vget /camera/1/lit png')
    img_left_rgb = read_png(res_l_rgb)

    # Camera 2: Right RGB
    res_r_rgb = client.request('vget /camera/2/lit png')
    img_right_rgb = read_png(res_r_rgb)

    # Camera 3: Center RGB
    res_c_rgb = client.request('vget /camera/3/lit png')
    img_center_rgb = read_png(res_c_rgb)

    # Camera 3: Center Depth
    # Using 'depth.planar' for better landscape geometry detection
    res_c_depth = client.request('vget /camera/3/depth npy')
    depth_center = read_npy(res_c_depth)

except Exception as e:
    print(f"Capture Error: {e}")

# --- DEPTH PROCESSING ---
if depth_center is not None:
    # Basic cleaning: Convert Infs/NaNs to 1000cm and clip the range
    depth_center = np.nan_to_num(depth_center, posinf=1000.0, nan=0.0, neginf=0.0)
    depth_center = np.clip(depth_center, 0, 1000.0)
    print(f"Depth captured. Min: {depth_center.min():.1f}cm, Max: {depth_center.max():.1f}cm")
else:
    print("Error: Could not retrieve depth data from Camera 3.")

# --- VISUALIZATION (2x2 Grid) ---
fig, axes = plt.subplots(2, 2, figsize=(12, 10))

# Row 1: Stereo RGB Pair
# Left RGB (Top Left)
if img_left_rgb is not None:
    axes[0, 0].imshow(img_left_rgb)
axes[0, 0].set_title('Camera 1: Left RGB')
axes[0, 0].axis('off')

# Right RGB (Top Right)
if img_right_rgb is not None:
    axes[0, 1].imshow(img_right_rgb)
axes[0, 1].set_title('Camera 2: Right RGB')
axes[0, 1].axis('off')

# Row 2: Fusion Center Data
# Center RGB (Bottom Left)
if img_center_rgb is not None:
    axes[1, 0].imshow(img_center_rgb)
axes[1, 0].set_title('Camera 3: Center RGB (Fusion)')
axes[1, 0].axis('off')

# Center Depth (Bottom Right)
if depth_center is not None:
    # vmin/vmax lock the scale from 0 to 10m
    im_depth = axes[1, 1].imshow(depth_center, cmap='jet', vmin=0, vmax=1000)
    axes[1, 1].set_title('Camera 3: Center Depth (10m Clamp)')
    
    # Add vertical colorbar
    cbar = fig.colorbar(im_depth, ax=axes[1, 1], orientation='vertical', pad=0.05, fraction=0.046)
    cbar.set_label('Distance (cm)')
axes[1, 1].axis('off')

plt.tight_layout()
plt.show()