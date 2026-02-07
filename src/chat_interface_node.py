#!/usr/bin/env python3
"""
Chat interface node for Phantom Bridge.
Handles bidirectional communication between web UI and robot entities.

This node runs on FastDDS (same as C++ bridge) and uses file-based IPC
to communicate with the Grounding DINO node (which runs on CycloneDDS
for Gazebo camera access).

File IPC paths (per entity):
  /tmp/dino_prompts_spot.json      - prompts for Spot entity
  /tmp/dino_prompts_drone.json     - prompts for Drone entity
  /tmp/dino_prompts_operator.json  - prompts for Operator entity
  /tmp/dino_results_spot/          - detection results for Spot
  /tmp/dino_results_drone/         - detection results for Drone
  /tmp/dino_results_operator/      - detection results for Operator
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from datetime import datetime
import json
import os
import glob
import tempfile


ENTITIES = ['spot', 'drone', 'operator']

PROMPTS_FILES = {
    'spot': '/tmp/dino_prompts_spot.json',
    'drone': '/tmp/dino_prompts_drone.json',
    'operator': '/tmp/dino_prompts_operator.json',
}

RESULTS_DIRS = {
    'spot': '/tmp/dino_results_spot',
    'drone': '/tmp/dino_results_drone',
    'operator': '/tmp/dino_results_operator',
}


class ChatInterfaceNode(Node):
    def __init__(self):
        super().__init__('chat_interface')

        # Declare parameters with defaults
        self.declare_parameter('spot_chat_topic', '/spot/chat')
        self.declare_parameter('drone_chat_topic', '/drone/chat')
        self.declare_parameter('operator_chat_topic', '/operator/chat')

        # Get parameters
        spot_topic = self.get_parameter('spot_chat_topic').value
        drone_topic = self.get_parameter('drone_chat_topic').value
        operator_topic = self.get_parameter('operator_chat_topic').value

        self.get_logger().info(f'Chat Interface Node starting (FastDDS + file IPC)...')
        self.get_logger().info(f'Spot topic: {spot_topic}')
        self.get_logger().info(f'Drone topic: {drone_topic}')
        self.get_logger().info(f'Operator topic: {operator_topic}')
        for entity in ENTITIES:
            self.get_logger().info(f'{entity} prompts: {PROMPTS_FILES[entity]}')
            self.get_logger().info(f'{entity} results: {RESULTS_DIRS[entity]}')

        # QoS profile matching C++ bridge (BEST_EFFORT, depth 1)
        bridge_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publishers - for sending messages to UI (via FastDDS -> C++ bridge -> WebRTC)
        self.chat_pubs = {
            'spot': self.create_publisher(String, spot_topic, bridge_qos),
            'drone': self.create_publisher(String, drone_topic, bridge_qos),
            'operator': self.create_publisher(String, operator_topic, bridge_qos),
        }

        # Subscribers - for receiving messages from UI (via WebRTC -> C++ bridge -> FastDDS)
        self.spot_sub = self.create_subscription(
            String, spot_topic, self.spot_callback, bridge_qos)
        self.drone_sub = self.create_subscription(
            String, drone_topic, self.drone_callback, bridge_qos)
        self.operator_sub = self.create_subscription(
            String, operator_topic, self.operator_callback, bridge_qos)

        # Active prompts per entity (managed via file IPC)
        self.active_prompts = {entity: [] for entity in ENTITIES}
        for entity in ENTITIES:
            self._load_prompts_from_file(entity)

        # Ensure results directories exist
        for results_dir in RESULTS_DIRS.values():
            os.makedirs(results_dir, exist_ok=True)

        # Timer to poll for detection results from DINO node (every 0.5s)
        self.results_timer = self.create_timer(0.5, self.poll_detection_results)

        # Send initial greeting
        self._send_message('spot', "Ready to assist Spot operations")
        self._send_message('drone', "Drone systems initialized")
        self._send_message('operator', "Operator interface ready")

    def _load_prompts_from_file(self, entity):
        """Load current prompts from the entity's shared file."""
        prompts_file = PROMPTS_FILES[entity]
        try:
            if os.path.exists(prompts_file):
                with open(prompts_file, 'r') as f:
                    self.active_prompts[entity] = json.load(f)
        except (json.JSONDecodeError, IOError):
            self.active_prompts[entity] = []

    def _save_prompts_to_file(self, entity):
        """Atomically write prompts to the entity's shared file."""
        prompts_file = PROMPTS_FILES[entity]
        try:
            # Write to temp file then rename for atomicity
            fd, tmp_path = tempfile.mkstemp(dir='/tmp', suffix='.json')
            with os.fdopen(fd, 'w') as f:
                json.dump(self.active_prompts[entity], f)
            os.replace(tmp_path, prompts_file)
        except IOError as e:
            self.get_logger().error(f'Failed to write prompts file for {entity}: {e}')

    def poll_detection_results(self):
        """Poll all entity result directories and forward to matching publisher."""
        for entity in ENTITIES:
            results_dir = RESULTS_DIRS[entity]
            try:
                result_files = sorted(glob.glob(os.path.join(results_dir, '*.json')))
            except OSError:
                continue

            for result_file in result_files:
                try:
                    with open(result_file, 'r') as f:
                        result_data = f.read()
                    # Remove the file after reading
                    os.unlink(result_file)
                    # Republish on the entity's topic so C++ bridge forwards to WebRTC
                    msg = String()
                    msg.data = result_data
                    self.chat_pubs[entity].publish(msg)
                    self.get_logger().info(f'Forwarded detection result to /{entity}/chat')
                except (IOError, OSError) as e:
                    self.get_logger().warn(f'Error reading result file {result_file}: {e}')

    def _handle_prompt_command(self, entity, msg):
        """Handle prompt add/remove commands from any entity's chat widget.
        Returns True if the message was a prompt command, False otherwise."""
        try:
            data = json.loads(msg.data)
            msg_type = data.get('type', '')

            if msg_type in ('add_prompt', 'remove_prompt'):
                prompt = data.get('prompt', '').strip()
                if prompt:
                    timestamp = datetime.now().strftime('%H:%M:%S')
                    if msg_type == 'add_prompt':
                        if prompt not in self.active_prompts[entity]:
                            self.active_prompts[entity].append(prompt)
                            self._save_prompts_to_file(entity)
                            self.get_logger().info(
                                f'[{timestamp}] {entity}: Added prompt "{prompt}" -> file IPC')
                    else:
                        if prompt in self.active_prompts[entity]:
                            self.active_prompts[entity].remove(prompt)
                            self._save_prompts_to_file(entity)
                            self.get_logger().info(
                                f'[{timestamp}] {entity}: Removed prompt "{prompt}" -> file IPC')
                return True

            # Detection results we republished - ignore to avoid echo loop
            if msg_type == 'detection':
                return True
        except (json.JSONDecodeError, TypeError):
            pass

        return False

    def spot_callback(self, msg):
        """Handle messages from Spot chat widget (via C++ bridge FastDDS)."""
        if self._handle_prompt_command('spot', msg):
            return
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Spot: {msg.data}')

    def drone_callback(self, msg):
        """Handle messages from Drone chat widget (via C++ bridge FastDDS)."""
        if self._handle_prompt_command('drone', msg):
            return
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Drone: {msg.data}')

    def operator_callback(self, msg):
        """Handle messages from Operator chat widget (via C++ bridge FastDDS)."""
        if self._handle_prompt_command('operator', msg):
            return
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Operator: {msg.data}')

    def _send_message(self, entity, text):
        """Send a message to the specified entity's chat widget."""
        msg = String()
        msg.data = text
        self.chat_pubs[entity].publish(msg)

    # Keep legacy helpers for backward compat if called externally
    def send_spot_message(self, text):
        self._send_message('spot', text)

    def send_drone_message(self, text):
        self._send_message('drone', text)

    def send_operator_message(self, text):
        self._send_message('operator', text)


def main():
    rclpy.init()
    node = ChatInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
