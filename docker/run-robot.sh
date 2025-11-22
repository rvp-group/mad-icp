#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${1:-mad-icp-robot}"
CONTAINER_NAME="mad-icp-robot"

# Stop existing container if it's already running
if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
  echo "Stopping existing container ${CONTAINER_NAME}..."
  docker rm -f "${CONTAINER_NAME}"
fi

echo "Starting robot container ${CONTAINER_NAME} from image ${IMAGE_NAME}..."

docker run \
  -d \
  --name "${CONTAINER_NAME}" \
  --restart unless-stopped \
  --network host \
  "${IMAGE_NAME}"
