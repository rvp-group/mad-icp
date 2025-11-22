#!/usr/bin/env bash
set -euo pipefail

# Allow Docker containers to connect to the X server (rviz2)
xhost +local:root >/dev/null 2>&1 || true

# Make sure these dirs exists on the host
sudo rm -rf build install log
mkdir -p build install log

docker compose -f docker-compose.dev.yml run --rm mad_icp_dev bash
