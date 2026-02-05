#!/usr/bin/env python3
"""
Chat interface node for Phantom Bridge.
Handles bidirectional communication between web UI and robot entities.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime


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
        
        self.get_logger().info(f'Chat Interface Node starting...')
        self.get_logger().info(f'Spot topic: {spot_topic}')
        self.get_logger().info(f'Drone topic: {drone_topic}')
        self.get_logger().info(f'Operator topic: {operator_topic}')
        
        # Publishers - for sending messages to UI
        self.spot_pub = self.create_publisher(String, spot_topic, 10)
        self.drone_pub = self.create_publisher(String, drone_topic, 10)
        self.operator_pub = self.create_publisher(String, operator_topic, 10)
        
        # Subscribers - for receiving messages from UI
        self.spot_sub = self.create_subscription(
            String, spot_topic, self.spot_callback, 10)
        self.drone_sub = self.create_subscription(
            String, drone_topic, self.drone_callback, 10)
        self.operator_sub = self.create_subscription(
            String, operator_topic, self.operator_callback, 10)
        
        # Example: Send initial greeting
        self.send_spot_message("Ready to assist Spot operations")
        self.send_drone_message("Drone systems initialized")
        self.send_operator_message("Operator interface ready")
    
    def spot_callback(self, msg):
        """Handle messages from Spot chat widget."""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Spot: {msg.data}')
        
        # You can add logic here to process Spot commands
        # For now, just log them
        
        # Optional: Send an acknowledgment
        # self.send_spot_message(f"Received: {msg.data}")
    
    def drone_callback(self, msg):
        """Handle messages from Drone chat widget."""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Drone: {msg.data}')
        
        # You can add logic here to process Drone commands
        # For now, just log them
        
        # Optional: Send an acknowledgment
        # self.send_drone_message(f"Received: {msg.data}")
    
    def operator_callback(self, msg):
        """Handle messages from Operator chat widget."""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.get_logger().info(f'[{timestamp}] Operator: {msg.data}')
        
        # You can add logic here to process Operator commands
        # For now, just log them
        
        # Optional: Send an acknowledgment
        # self.send_operator_message(f"Received: {msg.data}")
    
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
