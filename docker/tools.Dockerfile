# Interactive tools for debug_cli: the base image plus rviz2, rqt and the image
# viewers, shown on the host display. The components never need these.
#
# Built by docker compose (service debug_cli) after the base image.

FROM gnc_moon_simulator/base:1.0

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-image-view \
    ros-jazzy-image-transport-plugins \
    htop \
    nano \
 && rm -rf /var/lib/apt/lists/*

CMD ["bash"]
