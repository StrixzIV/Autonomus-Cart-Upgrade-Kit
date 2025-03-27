all: ros-image up

ros-image:
	docker build -t ros2_humble_acuk .

up:
	docker-compose up -d

down:
	docker-compose down

terminal:
	docker exec -it ros2_humble_acuk bash

restart: down up

re: down all up
