#!/bin/bash

# Change to the moveit2-py-docker directory
cd ~/moveit2-py-docker || exit 1

# Check if any container starting with moveit2 is already running
if docker ps --format '{{.Names}}' | grep -q "^moveit2"; then
    echo "Container starting with 'moveit2' is already running. Executing bash..."
    # Get the actual container name
    CONTAINER_NAME=$(docker ps --format '{{.Names}}' | grep "^moveit2" | head -1)
    docker exec -it "$CONTAINER_NAME" /bin/bash
else
    echo "No container starting with 'moveit2' is running. Starting with 'run'..."
    docker compose run --rm moveit2 /bin/bash
fi
