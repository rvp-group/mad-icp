.PHONY: build-dev build-robot dev-shell robot-build robot-run merge-compile

build-dev:
	./docker/build.sh dev

build-robot:
	./docker/build.sh robot

dev-shell: build-dev
	./docker/run-dev.sh

robot-build: build-robot

robot-run:
	./docker/run-robot.sh
