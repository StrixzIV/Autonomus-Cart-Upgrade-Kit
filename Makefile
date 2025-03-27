all: ros-image up

ros-image:
	docker build -t ros2-foxy-acuk .

up:
	docker-compose up -d

terminal:
	docker exec -it ros2_foxy_acuk bash
