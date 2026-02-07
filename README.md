# Phantom Bridge Client (VLM)

Fast WebRTC + Socket.io ROS2 Bridge for real-time data and video streaming, teleoperation, HRI, and remote robot monitoring.
Comes with Docker Container control for the host machine, CPU and Wi-Fi monitoring, and customizable [Web Interface](https://docs.phntm.io/bridge/ui/overview).

This fork adds **Vision-Language Model (VLM)** integration with Grounding DINO zero-shot object detection and multi-entity chat interfaces.

[See full documentation here](https://docs.phntm.io/bridge)

## Features
- ROS2 Node, Topic and Service discovery
- Fast streaming of binary ROS2 messages (in and out)
- Fast H.264 video streaming (pre-encoded FFmpeg frames)
- Image & CompressedImage messages encoded and streamed as H.264 video (sw, cuda or vaapi encoding)
- Docker container discovery and control
- Reliable ROS2 Service calls via Socket.io
- ROS2 runtime Parameters read/write API
- Extra ROS2 packages can be easily included for custom message and service type support
- **Grounding DINO Detection** -- Zero-shot object detection via detection prompts from the chat UI
- **Multi-Entity Chat Interfaces** -- Spot, Drone, and Operator chat widgets with detection prompt management, result rendering, and persistent history
- Robot's Wi-Fi signal monitoring, scan & roaming (via Agent)
- File retrieval from any running Docker container (via Agent)
- System load and Docker stats monitoring (via Agent)
- Connects P2P or via a TURN server when P2P link is not possible
- Multiple peers can connect to the same machine at very low extra CPU cost
- Works with Rosbag and Sims such as Gazebo or Webots
- Supported with ROS2 Humble and newer

## Architecture
![Infrastructure map](https://raw.githubusercontent.com/PhantomCybernetics/phntm_bridge_docs/refs/heads/main/img/Architecture_Client.svg)

### Detection Data Flow

```
Browser Chat Widget
    |  (add_prompt JSON via WebRTC write channel)
    v
C++ Bridge (FastDDS, Domain 0)
    |  (ROS2 topic: /spot/chat, /drone/chat, /operator/chat)
    v
chat_interface_node.py (FastDDS, Domain 0)
    |  (file IPC: /tmp/dino_prompts_{entity}.json)
    v
grounding_dino_node.py (CycloneDDS, Domain 42)
    |  (reads camera images + prompts, runs inference)
    |  (file IPC: /tmp/dino_results_{entity}/*.json)
    v
chat_interface_node.py (FastDDS, Domain 0)
    |  (polls result files, publishes on /{entity}/chat)
    v
C++ Bridge -> WebRTC -> Browser (renders annotated image + detections)
```

## Quick Start

The fastest way to get started is to use the automated setup script:

```bash
./setup.sh
```

The setup script will:
1. Check Docker, Docker Compose, and wget/curl installation
2. Register your robot with the Bridge Server
3. Build the Docker image
4. Start the services
5. Provide you with your unique Bridge URL

**That's it!** Once complete, open `https://bridge.phntm.io/%YOUR_ID_ROBOT%` in your browser.

---

## Prerequisites

- **Docker** -- [Install Docker](https://docs.docker.com/engine/install/)
- **Docker Compose** -- Plugin (`docker compose`) or standalone (`docker-compose`)
- **git** -- For cloning the repository
- **wget** or **curl** -- For downloading the robot config

Make sure your root SSL certificates are up to date:
```bash
sudo apt update && sudo apt install ca-certificates
```

Add your user to the docker group:
```bash
sudo usermod -aG docker ${USER}
# Log out and back in
```

## Manual Setup

### 1. Clone the repository

```bash
git clone git@github.com:PhantomCybernetics/phntm_bridge_client.git
cd phntm_bridge_client
```

### 2. Register a new robot

This registers a new robot on the Bridge Server and returns a config file with unique `id_robot` and `key` credentials:

```bash
wget -O ~/phntm_bridge.yaml 'https://register.phntm.io/robot?yaml'
```

### 3. Build the Docker image

```bash
ROS_DISTRO=humble docker compose build
```

Or use a pre-built image from [ghcr.io/phantomcybernetics/phntm_bridge_client](https://github.com/PhantomCybernetics/phntm_bridge_client/pkgs/container/phntm_bridge_client).

### 4. Launch

```bash
docker compose up phntm_bridge
```

This launches the bridge client, agent, chat interface node, Grounding DINO detection node, and the chat widgets server.

### 5. Open the Web UI

Navigate to `https://bridge.phntm.io/%YOUR_ID_ROBOT%` in a web browser. The exact link can be found at the top of your `~/phntm_bridge.yaml` config file.

## Configuration

Config file: `~/phntm_bridge.yaml` (mounted as `/ros2_ws/phntm_bridge_params.yaml` in the container).

Full list of configuration options: [docs.phntm.io/bridge/basics/configuration](https://docs.phntm.io/bridge/basics/configuration)

### Camera Topics for Detection

By default, Spot cameras are auto-discovered from CycloneDDS domain 42 (e.g. Gazebo simulation cameras). To add drone or operator camera topics for entity-specific detection, edit `phntm_bridge.yaml`:

```yaml
/**:
  ros__parameters:
    # ...existing config...

    ## Camera topics for each entity (used by Grounding DINO detection)
    ## Spot cameras are auto-discovered from CycloneDDS domain 42
    ## Set drone/operator camera topics when available
    # drone_camera_topic: '/drone/camera/image_raw'
    # operator_camera_topic: '/operator/camera/image_raw'
```

Uncomment and set the topic name to enable entity-specific camera detection. When a drone or operator camera topic is configured, detections from that camera are routed to the corresponding chat widget.

## Chat Interfaces

Three chat widgets are available in the web UI (accessible from the Widgets menu):

| Widget | Topic | Purpose |
|--------|-------|---------|
| Chat: Spot | `/spot/chat` | Detection prompts for Spot's cameras |
| Chat: Drone | `/drone/chat` | Detection prompts for Drone's cameras |
| Chat: Operator | `/operator/chat` | Detection prompts for Operator's cameras |

### Using Detection Prompts

1. Open a chat widget from the Widgets menu in the web UI
2. Type a detection prompt in the input field (e.g. "red ball", "person", "door")
3. Press Enter or click Send -- the prompt appears as a chip above the input
4. Grounding DINO processes camera frames against active prompts
5. Detection results (annotated images with bounding boxes) appear in the chat
6. Click the X on a prompt chip to remove it

Each entity maintains its own set of active prompts. Prompts persist across browser sessions via localStorage.

### Cross-Domain DDS Architecture

The bridge uses two DDS domains to work around FastDDS wire-incompatibility between ROS2 distros:

- **Domain 0 (FastDDS)**: C++ bridge, agent, chat_interface_node -- handles WebRTC and introspection
- **Domain 42 (CycloneDDS)**: grounding_dino_node, Gazebo simulation -- handles camera data from Jazzy-based sims

File-based IPC bridges the gap between domains:
- Prompts: `/tmp/dino_prompts_{spot,drone,operator}.json`
- Results: `/tmp/dino_results_{spot,drone,operator}/*.json`

## Docker Compose

Add the phntm_bridge service to your `compose.yaml`:

> **Note:** Cyclone DDS is recommended over the default Fast DDS.

```yaml
services:
  phntm_bridge:
    image: ghcr.io/phantomcybernetics/phntm_bridge_client:main-humble
    container_name: phntm-bridge
    hostname: phntm-bridge.local
    restart: unless-stopped
    privileged: true
    network_mode: host
    ipc: host
    volumes:
      - ~/phntm_bridge.yaml:/ros2_ws/phntm_bridge_params.yaml
      - ~/phntm_bridge.yaml:/ros2_ws/phntm_agent_params.yaml
      - /var/run:/host_run
      - /tmp:/tmp
    devices:
      - /dev:/dev
    command:
      ros2 launch phntm_bridge client_agent_launch.py
```

## Troubleshooting

### Chat widgets not appearing

1. Hard refresh the browser: `Ctrl+Shift+R` (Windows/Linux) or `Cmd+Shift+R` (Mac)
2. Check the chat server is running:
   ```bash
   docker exec phntm-bridge curl -s http://localhost:3080/health
   ```
3. Check container logs:
   ```bash
   docker logs phntm-bridge
   ```

### Detection prompts not producing results

1. Verify Grounding DINO node is running:
   ```bash
   docker logs phntm-bridge 2>&1 | grep -i "grounding dino"
   ```
2. Check that camera topics are being discovered:
   ```bash
   docker logs phntm-bridge 2>&1 | grep "Subscribing to camera"
   ```
3. Ensure prompts file is being written:
   ```bash
   docker exec phntm-bridge cat /tmp/dino_prompts_spot.json
   ```

### Mixed content / connection refused errors

The chat server runs inside the container on `localhost:3080`. It is accessed from the browser via the WebRTC bridge, not directly. If you see connection errors, the container may still be starting up -- wait a moment and retry.

### Gazebo simulation cameras not detected

- Ensure Gazebo sim is running with `ROS_DOMAIN_ID=42` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- The grounding_dino_node auto-discovers `sensor_msgs/msg/Image` topics on domain 42
- Check with: `ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 topic list`

## Upgrading

```bash
docker stop phntm-bridge && docker rm phntm-bridge && docker image rm phntm/bridge:humble
cd ~/phntm_bridge_client
git pull
ROS_DISTRO=humble docker compose build
docker compose up phntm_bridge
```

## See Also

- [Documentation](https://docs.phntm.io/bridge) Full Phantom Bridge documentation
- [Bridge UI](https://docs.phntm.io/bridge/ui/overview) Overview of the customizable web UI
- [Configure User Input](https://docs.phntm.io/bridge/ui/user-input-and-teleoperation) Use keyboard, touch or gamepad to control your robot
- [Picam ROS2](https://github.com/PhantomCybernetics/picam_ros2) Standalone ROS2 node for hardware-encoded H.264 frames
