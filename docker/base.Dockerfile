FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Base tools & certs
RUN apt-get update && apt-get install -y \
    ca-certificates gnupg lsb-release curl \
    tmux git build-essential python3-colcon-common-extensions python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

# (Optional) Add repo.ros2.org extra source (signed)
RUN install -m 0755 -d /usr/share/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://repo.ros2.org/ubuntu/main $(lsb_release -sc) main" \
      > /etc/apt/sources.list.d/ros2-extra.list

# Install keys for Gazebo dependencies
RUN apt-get update && apt-get install -y curl gnupg lsb-release \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \ 
    > /etc/apt/sources.list.d/gazebo-stable.list

# Install ROS 2 packages available via APT on Jazzy
RUN apt-get update && apt-get install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joy-linux \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-depth-image-proc \
    ros-jazzy-rplidar-ros \
    ros-jazzy-ament-cmake-black \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rviz2 \ 
    ros-jazzy-rviz-common \ 
    ros-jazzy-rviz-default-plugins \
    libogre-1.12-dev binutils \
    libgz-math7-dev \
    python3-pip \
    ros-jazzy-image-transport-plugins \
    libspdlog-dev \
    libfmt-dev \
    htop \
    nano \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ros/jazzy/src
RUN rm -rf ./*

CMD ["bash"]