#!/usr/bin/env bash
# Builds the base image, then every component image, and starts the stack.
# Stops at the first failure: a component built on a missing or stale base
# image would fail later in a less obvious way.
set -euo pipefail
cd "$(dirname "$0")"

docker build -t gnc_moon_simulator/base:1.0 -f docker/base.Dockerfile docker
docker compose build
docker compose up -d
