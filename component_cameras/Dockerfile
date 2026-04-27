FROM gnc_moon_simulator/base:1.0

# ---- Runtime entrypoint ----
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN python3 -m pip install --ignore-installed unrealcv "numpy<2.0.0" --break-system-packages
RUN python3 -m pip install matplotlib --break-system-packages
RUN apt-get update && apt-get install -y python3-tk

# Defaults (override with -e at runtime)
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    VERBOSITY=-v4

CMD ["bash", "/entrypoint.sh"]