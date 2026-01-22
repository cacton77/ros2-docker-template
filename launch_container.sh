#!/bin/bash

# Change to the moveit2-py-docker directory
cd ~/ros2-docker-template || exit 1

# Check if any container starting with moveit2 is already running
if docker ps --format '{{.Names}}' | grep -q "^ros2-docker-template"; then
    echo "Container starting with 'ros2-docker-template' is already running. Executing bash..."
    # Get the actual container name
    CONTAINER_NAME=$(docker ps --format '{{.Names}}' | grep "^ros2-docker-template" | head -1)
    docker exec -it "$CONTAINER_NAME" /bin/bash
else
    echo "No container starting with 'ros2-docker-template' is running. Starting with 'run'..."
    docker compose run --rm ros2-docker-template /bin/bash
fi
