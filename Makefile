all: ros-image up

ros-image:
	docker build -t ros2_humble_acuk .

up:
	docker-compose up -d

terminal:
	docker exec -it ros2_humble_acuk bash

clean:
	docker-compose down
	docker system prune -f

restart: down up

re: clean all

.PHONY: all ros-image up down terminal down restart re