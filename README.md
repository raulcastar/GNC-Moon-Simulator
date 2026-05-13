# GNC-Moon-Simulator


## ⚙️ WSL2 System Configuration (`.wslconfig`)

Here is the exact file path:
C:\Users\<YourUsername>\.wslconfig

(Make sure to replace <YourUsername> with your actual Windows login name).

```ini
[wsl2]
# Merges the WSL network with the Windows network
networkingMode=mirrored

# Ensures localhost ports are shared between Windows and WSL
localhostForwarding=true

# (Optional) Forces WSL to use the exact same DNS as Windows
dnsTunneling=true