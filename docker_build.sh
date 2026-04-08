docker build -t gnc_moon_simulator/base:1.0 -f docker/base.Dockerfile docker

# Build all child images and start
docker compose build
docker compose up -d