CUR_DIR = $(shell pwd)
IMAGE_NAME = $$(basename $(CUR_DIR))-duckiebot-image

build:
	docker build -t "$(IMAGE_NAME)" --progress=plain .
	docker run --rm -v ./:/workspace --entrypoint=colcon "$(IMAGE_NAME)" build

run:
	docker run --rm -it \
		--network=host \
		--privileged \
		-e VEHICLE_NAME=duckie04 \
		-v /dev/shm:/dev/shm \
		-v ./:/workspace \
		"$(IMAGE_NAME)" \
		/bin/bash -c "source /workspace/install/setup.bash && exec bash"

clean:
	docker run --rm -it --network=host --privileged -v /dev/shm:/dev/shm -v ./:/workspace "$(IMAGE_NAME)" \
	/bin/bash -c "rm -rf log build install && echo 'Clean complete'"

.PHONY: run build clean
