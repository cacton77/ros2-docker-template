#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR" || exit 1

# Load environment variables
if [ -f .env ]; then
    source .env
fi

# Default container name if not set
CONTAINER_NAME="${CONTAINER_NAME:-ros2-docker-template}"

# Build compose command based on GPU setting
COMPOSE_CMD="docker compose -f docker-compose.yaml"
if [ "$USE_GPU" = "true" ]; then
    COMPOSE_CMD="$COMPOSE_CMD -f docker-compose.nvidia.yaml"
fi

# Check if running interactively (has a TTY)
if [ -t 0 ] && [ -t 1 ]; then
    # Interactive mode - original behavior
    RUNNING_CONTAINER=$(docker ps --format '{{.Names}}' | grep -E "^${CONTAINER_NAME}(-app-run-.*)?$" | head -n 1)

    if [ -n "$RUNNING_CONTAINER" ]; then
        echo "Container '$RUNNING_CONTAINER' is already running. Attaching..."
    else
        echo "Starting container '$CONTAINER_NAME'..."
        if [ "$USE_GPU" = "true" ]; then
            echo "GPU support: enabled"
        else
            echo "GPU support: disabled"
        fi
        $COMPOSE_CMD up -d app
        echo "Container started in detached mode."
        RUNNING_CONTAINER="$CONTAINER_NAME"
        # Wait for container to be ready
        sleep 2
    fi

    # Attach interactively
    docker exec -it "$RUNNING_CONTAINER" bash -ic "ros2 launch inspection_eoat bringup.launch.py"
else
    # Non-interactive mode (systemd service)
    echo "Starting container '$CONTAINER_NAME' in service mode..."
    if [ "$USE_GPU" = "true" ]; then
        echo "GPU support: enabled"
    fi

    # Stop any existing container first
    $COMPOSE_CMD down --remove-orphans 2>/dev/null || true

    # Run in foreground with the ROS2 launch command
    # This keeps the process running so systemd can manage it
    $COMPOSE_CMD run --rm --name "$CONTAINER_NAME" app \
        bash -c "source /opt/ros/humble/setup.bash && source /workspaces/shared_ws/install/setup.bash 2>/dev/null; ros2 launch inspection_eoat bringup.launch.py"
fi
