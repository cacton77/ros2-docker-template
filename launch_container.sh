#!/bin/bash

# Change to the ros2-docker-template directory
cd ~/ros2-docker-template || exit 1

# Load environment variables
if [ -f .env ]; then
    source .env
fi

# Build compose command based on GPU setting
COMPOSE_CMD="docker compose -f docker-compose.yaml"
if [ "$USE_GPU" = "true" ]; then
    COMPOSE_CMD="$COMPOSE_CMD -f docker-compose.nvidia.yaml"
fi

# Check if any container starting with ros2-docker-template is already running
if docker ps --format '{{.Names}}' | grep -q "^ros2-docker-template"; then
    echo "Container starting with 'ros2-docker-template' is already running. Executing bash..."
    # Get the actual container name
    CONTAINER_NAME=$(docker ps --format '{{.Names}}' | grep "^ros2-docker-template" | head -1)
    docker exec -it "$CONTAINER_NAME" /bin/bash
else
    echo "No container starting with 'ros2-docker-template' is running. Starting with 'run'..."
    if [ "$USE_GPU" = "true" ]; then
        echo "GPU support enabled"
    else
        echo "GPU support disabled"
    fi
    $COMPOSE_CMD run --rm ros2-docker-template /bin/bash
fi
