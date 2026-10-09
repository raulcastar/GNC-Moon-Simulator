FROM gnc_moon_simulator/base:1.0

# python3-tk is required for the TkAgg matplotlib backend (live DEM debug window)
RUN apt-get update && apt-get install -y --no-install-recommends python3-tk && \
    rm -rf /var/lib/apt/lists/*

# What the shared entrypoint (base image) launches.
ENV GNC_PACKAGE=component_local_mapping

CMD ["bash", "/entrypoint.sh"]
