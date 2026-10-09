FROM gnc_moon_simulator/base:1.0

RUN python3 -m pip install --ignore-installed "numpy<2.0.0" --break-system-packages

# stereo_msgs ships with common_interfaces, but pull it in explicitly so the
# DisparityImage publisher cannot break on a slimmer base image.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-stereo-msgs && \
    rm -rf /var/lib/apt/lists/*

# What the shared entrypoint (base image) launches.
ENV GNC_PACKAGE=component_stereo_depth

CMD ["bash", "/entrypoint.sh"]
