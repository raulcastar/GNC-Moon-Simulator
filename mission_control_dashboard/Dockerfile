FROM gnc_moon_simulator/base:1.0

# ---- Runtime entrypoint ----
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Defaults (override with -e at runtime)
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    VERBOSITY=-v4

RUN python3 -m pip install --ignore-installed \
    Flask==3.0.3 \
    Werkzeug==3.0.3 \
    Flask-SocketIO==5.3.6 \
    python-engineio==4.9.0 \
    python-socketio==5.11.2 \
    simple-websocket==1.0.0 \
    --break-system-packages

CMD ["bash", "/entrypoint.sh"]