#!/usr/bin/env python3
"""
Chat interface node for Phantom Bridge.
Handles bidirectional communication between web UI and robot entities.

This node runs on FastDDS (same as C++ bridge) and uses file-based IPC
to communicate with the Grounding DINO node (which runs on CycloneDDS
for Gazebo camera access).

File IPC paths:
  /tmp/dino_prompts.json  - prompts written by this node, read by DINO
  /tmp/dino_results/      - detection JSONs written by DINO, read by this node
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


PROMPTS_FILE = '/tmp/dino_prompts.json'
RESULTS_DIR = '/tmp/dino_results'


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
        self.get_logger().info(f'Prompts file: {PROMPTS_FILE}')
        self.get_logger().info(f'Results dir: {RESULTS_DIR}')

        # QoS profile matching C++ bridge (BEST_EFFORT, depth 1)
        bridge_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publishers - for sending messages to UI (via FastDDS -> C++ bridge -> WebRTC)
        self.spot_pub = self.create_publisher(String, spot_topic, bridge_qos)
        self.drone_pub = self.create_publisher(String, drone_topic, bridge_qos)
        self.operator_pub = self.create_publisher(String, operator_topic, bridge_qos)

        # Subscribers - for receiving messages from UI (via WebRTC -> C++ bridge -> FastDDS)
        self.spot_sub = self.create_subscription(
            String, spot_topic, self.spot_callback, bridge_qos)
        self.drone_sub = self.create_subscription(
            String, drone_topic, self.drone_callback, bridge_qos)
        self.operator_sub = self.create_subscription(
            String, operator_topic, self.operator_callback, bridge_qos)

        # Active prompts list (managed via file IPC)
        self.active_prompts = []
        self._load_prompts_from_file()

        # Ensure results directory exists
        os.makedirs(RESULTS_DIR, exist_ok=True)

        # Timer to poll for detection results from DINO node (every 0.5s)
        self.results_timer = self.create_timer(0.5, self.poll_detection_results)

        # Send initial greeting
        self.send_spot_message("Ready to assist Spot operations")
        self.send_drone_message("Drone systems initialized")
        self.send_operator_message("Operator interface ready")

    def _load_prompts_from_file(self):
        """Load current prompts from the shared file."""
        try:
            if os.path.exists(PROMPTS_FILE):
                with open(PROMPTS_FILE, 'r') as f:
                    self.active_prompts = json.load(f)
        except (json.JSONDecodeError, IOError):
            self.active_prompts = []

    def _save_prompts_to_file(self):
        """Atomically write prompts to the shared file."""
        try:
            # Write to temp file then rename for atomicity
            fd, tmp_path = tempfile.mkstemp(dir='/tmp', suffix='.json')
            with os.fdopen(fd, 'w') as f:
                json.dump(self.active_prompts, f)
            os.replace(tmp_path, PROMPTS_FILE)
        except IOError as e:
            self.get_logger().error(f'Failed to write prompts file: {e}')

    def poll_detection_results(self):
        """Poll /tmp/dino_results/ for detection result files and republish on FastDDS."""
        try:
            result_files = sorted(glob.glob(os.path.join(RESULTS_DIR, '*.json')))
        except OSError:
            return

        for result_file in result_files:
            try:
                with open(result_file, 'r') as f:
                    result_data = f.read()
                # Remove the file after reading
                os.unlink(result_file)
                # Republish on FastDDS /spot/chat so C++ bridge forwards to WebRTC
                msg = String()
                msg.data = result_data
                self.spot_pub.publish(msg)
                self.get_logger().info(f'Forwarded detection result to /spot/chat')
            except (IOError, OSError) as e:
                self.get_logger().warn(f'Error reading result file {result_file}: {e}')

    def spot_callback(self, msg):
        """Handle messages from Spot chat widget (via C++ bridge FastDDS)."""
        timestamp = datetime.now().strftime('%H:%M:%S')

        # Try to parse as JSON prompt command from web UI
        try:
            data = json.loads(msg.data)
            msg_type = data.get('type', '')

            if msg_type in ('add_prompt', 'remove_prompt'):
                prompt = data.get('prompt', '').strip()
                if prompt:
                    if msg_type == 'add_prompt':
                        if prompt not in self.active_prompts:
                            self.active_prompts.append(prompt)
                            self._save_prompts_to_file()
                            self.get_logger().info(f'[{timestamp}] Added prompt "{prompt}" -> file IPC')
                    else:
                        if prompt in self.active_prompts:
                            self.active_prompts.remove(prompt)
                            self._save_prompts_to_file()
                            self.get_logger().info(f'[{timestamp}] Removed prompt "{prompt}" -> file IPC')
                return

            # Detection results we republished - ignore to avoid echo loop
            if msg_type == 'detection':
                return
        except (json.JSONDecodeError, TypeError):
            pass

        # Regular text message
        self.get_logger().info(f'[{timestamp}] Spot: {msg.data}')

    def drone_callback(self, msg):
        """Handle messages from Drone chat widget."""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Drone: {msg.data}')

    def operator_callback(self, msg):
        """Handle messages from Operator chat widget."""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Operator: {msg.data}')

    def send_spot_message(self, text):
        """Send a message to the Spot chat widget."""
        msg = String()
        msg.data = text
        self.spot_pub.publish(msg)

    def send_drone_message(self, text):
        """Send a message to the Drone chat widget."""
        msg = String()
        msg.data = text
        self.drone_pub.publish(msg)

    def send_operator_message(self, text):
        """Send a message to the Operator chat widget."""
        msg = String()
        msg.data = text
        self.operator_pub.publish(msg)


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
