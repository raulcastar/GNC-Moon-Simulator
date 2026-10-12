# Base image of every GNC component container (and the Zenoh router).
#
# Headless and minimal: ros:jazzy (ROS 2 base, colcon, compilers for the
# workspace build each container runs at start) plus only what the nodes
# import. Desktop tools (rviz2, rqt) live in tools.Dockerfile, used by debug_cli.
#
#   docker build -t gnc_moon_simulator/base:1.0 -f docker/base.Dockerfile docker

FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Middleware: every node and the router speak Zenoh
    ros-jazzy-rmw-zenoh-cpp \
    # Images: cv_bridge (and OpenCV) for the cameras, stereo and local mapping
    ros-jazzy-cv-bridge \
    python3-opencv \
    # Point clouds (lidar, local mapping)
    ros-jazzy-sensor-msgs-py \
    # Rosbags recorded by the dashboard are MCAP
    ros-jazzy-rosbag2-storage-mcap \
    # Imported by the planner and the local mapper (optional debug plots)
    python3-matplotlib \
    # Unit tests (./run_tests.sh runs them inside the containers)
    python3-pytest \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ros/jazzy/src

# Shared component entrypoint: each component image sets GNC_PACKAGE (and
# GNC_LAUNCH when its launch file is not lc_<package>.launch.py).
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp

CMD ["bash"]
