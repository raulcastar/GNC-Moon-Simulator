FROM gnc_moon_simulator/base:1.0

RUN python3 -m pip install --ignore-installed \
    Flask==3.0.3 \
    Werkzeug==3.0.3 \
    Flask-SocketIO==5.3.6 \
    python-engineio==4.9.0 \
    python-socketio==5.11.2 \
    simple-websocket==1.0.0 \
    --break-system-packages

# What the shared entrypoint (base image) launches.
ENV GNC_PACKAGE=mission_control_dashboard \
    GNC_LAUNCH=mission_control_dashboard.launch.py

CMD ["bash", "/entrypoint.sh"]
