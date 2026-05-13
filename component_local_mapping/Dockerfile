FROM gnc_moon_simulator/base:1.0

# python3-tk is required for the TkAgg matplotlib backend (live DEM debug window)
RUN apt-get update && apt-get install -y --no-install-recommends python3-tk && \
    rm -rf /var/lib/apt/lists/*

# ---- Runtime entrypoint ----
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Defaults (override with -e at runtime)
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    VERBOSITY=-v4

CMD ["bash", "/entrypoint.sh"]
