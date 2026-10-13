"""Project startup script (Project Settings > Python > Startup Scripts): the
viewport camera follows the mission control dashboard (rover_cameras.py)."""
import rover_cameras

rover_cameras.start_dashboard_link()
