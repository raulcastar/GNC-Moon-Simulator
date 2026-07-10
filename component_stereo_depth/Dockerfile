FROM gnc_moon_simulator/base:1.0

# ---- Runtime entrypoint ----
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN python3 -m pip install --ignore-installed unrealcv "numpy<2.0.0" --break-system-packages

# stereo_msgs ships with common_interfaces, but pull it in explicitly so the
# DisparityImage publisher cannot break on a slimmer base image.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-stereo-msgs && \
    rm -rf /var/lib/apt/lists/*

# Defaults (override with -e at runtime)
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    VERBOSITY=-v4

CMD ["bash", "/entrypoint.sh"]
