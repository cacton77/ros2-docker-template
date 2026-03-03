# ROS2 Docker Template

A general-purpose Docker development template for ROS2 projects. Clone this repo, configure your packages, and get a consistent, reproducible ROS2 workspace with optional Arduino firmware support.

---

## Overview

The container is built on `osrf/ros:humble-desktop-full` and includes:

- **ROS2 Humble** with a colcon workspace (`src/`) bind-mounted from the host
- **CycloneDDS** as the ROS2 middleware
- **Arduino CLI** installed locally for RP2040 firmware compile and flash workflows
- **Desktop shortcuts** for one-click bringup and development shells
- **NVIDIA GPU** passthrough support (auto-detected or overridden via flags)

---

## Repository Structure

```
ros2-docker-template/
├── .env                          # Container name, ROS domain ID, GPU flag, firmware/run settings
├── install.sh                    # One-time setup and rebuild script
├── launch_container.sh           # Start or attach to the container
├── run.sh                        # Run RUN_CMD (from .env) inside the container
├── docker-compose.yaml           # Main Docker Compose file
├── docker-compose.nvidia.yaml    # NVIDIA GPU overlay (merged when USE_GPU=true)
├── docker/
│   ├── Dockerfile                # Image definition
│   ├── entrypoint.sh             # Sources ROS2, base_ws, and shared_ws on shell startup
│   ├── bringup_entrypoint.sh     # Entrypoint variant for bringup launches
│   ├── packages.txt              # APT packages installed into the image
│   └── requirements.txt          # Python pip packages
├── config/
│   └── cyclonedds.xml            # CycloneDDS tuning (buffer sizes, multicast)
├── src/
│   └── src.repos                 # vcstool manifest — add your ROS2 packages here
├── data/
│   └── data.repos                # vcstool manifest for large data repositories (Git LFS)
├── desktop/
│   ├── bringup.desktop           # Desktop shortcut: Bringup
│   └── devel.desktop             # Desktop shortcut: Development shell
├── assets/
│   ├── bringup_icon.png
│   └── devel_icon.png
└── arduino/
    ├── arduino-cli.yaml          # Arduino CLI config (self-contained, paths relative to arduino/)
    ├── cores.txt                 # Board cores to install (one per line)
    └── libraries.txt             # Libraries to install (one per line)
```

---

## Prerequisites

- [Docker](https://docs.docker.com/engine/install/) with the Compose plugin
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) (if using GPU)

---

## Getting Started

### 1. Clone and rename

Clone this template and rename the folder to your project name. The container name is automatically derived from the folder name (lowercased, spaces replaced with hyphens).

### 2. Configure your ROS2 packages

Edit [src/src.repos](src/src.repos) to list the repositories to clone into the workspace:

```yaml
repositories:
    my_package:
        type: git
        url: https://github.com/your-org/my_package
        version: main
```

### 3. Configure data repositories (optional)

If your project requires large data assets stored in Git LFS, edit [data/data.repos](data/data.repos) to list the repositories to clone into `data/`:

```yaml
repositories:
    MyDataset:
        type: git
        url: https://github.com/your-org/MyDataset
        version: main
```

`install.sh` will clone each repository and run `git lfs pull` inside it. If you have no data repositories, leave `data/data.repos` empty.

### 4. Configure `.env`

| Variable | Default | Description |
|---|---|---|
| `CONTAINER_NAME` | *(folder name)* | Docker container/image name (set automatically by `install.sh`) |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain ID |
| `USE_GPU` | `true` | Enable NVIDIA GPU passthrough (set automatically by `install.sh`) |
| `FIRMWARE_DIR` | *(empty)* | Path to Arduino sketch directory (relative or absolute); leave empty to skip firmware steps |
| `RUN_CMD` | `echo "Hello World"` | Command run by `run.sh` inside the container |

### 5. Install

```bash
./install.sh
```

The script will:
1. Install `vcstool` if not present
2. Clone all packages from `src/src.repos` into `src/`
3. Install `git-lfs` if not present and clone data repositories from `data/data.repos`
4. Auto-detect an NVIDIA GPU (override with `--gpu` or `--no-gpu`)
5. Build (or rebuild) the Docker image
6. Copy app files and create desktop shortcuts (`bringup.desktop`, `devel.desktop`) on `~/Desktop`
7. Install Arduino CLI locally into `arduino/bin/` if not already present
8. Install board cores listed in `arduino/cores.txt` and libraries in `arduino/libraries.txt`
9. If `FIRMWARE_DIR` is set: compile and flash the Arduino sketch to a connected RP2040
10. Increase kernel socket buffer limits for DDS performance
11. Build the ROS2 workspace inside the container

```bash
# Force GPU support
./install.sh --gpu

# Disable GPU support
./install.sh --no-gpu
```

Re-running `install.sh` during development rebuilds the Docker image (using cache) and rebuilds the workspace.

---

## Usage

### Interactive Development Shell

Open a bash shell inside the container (starts the container if not already running):

```bash
./launch_container.sh
```

If the container is already running, this attaches a new shell to it. Additional arguments are passed as a command:

```bash
./launch_container.sh colcon build
./launch_container.sh ros2 topic list
```

### Run a Command

`run.sh` reads `RUN_CMD` from `.env` and runs it inside the container:

```bash
./run.sh
```

Set `RUN_CMD` to your bringup launch, test script, or any other command.

### Desktop Shortcuts

After running `install.sh`, two shortcuts appear on the desktop:

- **Bringup** — launches `RUN_CMD` in a terminal
- **Development** — opens an interactive development shell

---

## Workspace Layout Inside the Container

| Path | Description |
|---|---|
| `/workspaces/base_ws` | Empty base workspace (built into the image layer) |
| `/workspaces/shared_ws` | Your ROS2 packages (bind-mounted from `./src`) |
| `/data` | Data repositories (bind-mounted from `./data`) |
| `/config` | Runtime config including CycloneDDS (bind-mounted from `./config`) |

The entrypoint sources workspaces in order: ROS2 → `base_ws` → `shared_ws`.

---

## Arduino Firmware Support

Arduino CLI is installed locally into `arduino/bin/` (not added to the system PATH). All Arduino data, downloads, and user libraries are stored under `arduino/` to keep the project self-contained.

**Configuring cores and libraries:**

Add board cores to [arduino/cores.txt](arduino/cores.txt) (one per line):
```
rp2040:rp2040
```

Add libraries to [arduino/libraries.txt](arduino/libraries.txt) (one per line):
```
Adafruit NeoPixel
```

**Compiling and flashing firmware:**

Set `FIRMWARE_DIR` in `.env` to the path of your Arduino sketch (the folder containing the `.ino` file). Re-run `install.sh` to compile and flash. The script will:

1. Compile the sketch for the `rp2040:rp2040:adafruit_qtpy` board
2. Auto-detect a connected RP2040 (via serial port or RPI-RP2 bootloader drive)
3. Reset the board into bootloader mode if needed and copy the `.uf2` firmware

---

## CycloneDDS

[config/cyclonedds.xml](config/cyclonedds.xml) is mounted into the container at `/config/cyclonedds.xml` and configures CycloneDDS with tuned socket buffer sizes (25 MB send/receive) for high-bandwidth topics.

---

## Updating Packages

Pull the latest changes for all packages in `src/`:

```bash
cd src
vcs pull
```

Then rebuild the workspace:

```bash
./launch_container.sh colcon build
```

Or re-run `install.sh` for a full rebuild including the Docker image.
