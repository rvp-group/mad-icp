#!/usr/bin/env bash

IMAGE_NAME="mad-icp"
TARGET="${1:-dev}" # Defaults to dev target

case "$TARGET" in
dev)
  IMAGE_NAME="${IMAGE_NAME}-dev"
  DOCKER_TARGET="dev"
  ;;
robot)
  IMAGE_NAME="${IMAGE_NAME}-robot"
  DOCKER_TARGET="robot"
  ;;
*)
  echo "Usage: $0 [dev|robot]" >&2
  exit 1
  ;;
esac

echo "Building docker image: ${IMAGE_NAME} (target: ${DOCKER_TARGET})"

docker build \
  -t "${IMAGE_NAME}" \
  --target "${DOCKER_TARGET}" \
  -f Dockerfile .

echo "Done."
