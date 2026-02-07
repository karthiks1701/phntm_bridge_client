#!/usr/bin/env python3
"""
Camera relay publisher for Phantom Bridge.
Reads raw Image frames from /dev/shm/cam_relay/ (written by grounding_dino_node
on CycloneDDS domain 42) and republishes them on FastDDS domain 0 so the
C++ bridge can discover and stream them via WebRTC.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import os
import struct
import time


CAM_RELAY_DIR = '/dev/shm/cam_relay'


class CameraRelayPub(Node):
    def __init__(self):
        super().__init__('camera_relay')

        # Map: safe_name -> (publisher, last_mtime)
        self.cam_pubs = {}

        # Poll for new frames at ~5 Hz
        self.timer = self.create_timer(0.2, self.poll_frames)
        self.get_logger().info(f'Camera relay publisher ready (reading from {CAM_RELAY_DIR})')

    def poll_frames(self):
        try:
            files = os.listdir(CAM_RELAY_DIR)
        except OSError:
            return

        for filename in files:
            if not filename.endswith('.bin'):
                continue

            filepath = os.path.join(CAM_RELAY_DIR, filename)
            safe_name = filename[:-4]  # strip .bin

            try:
                mtime = os.path.getmtime(filepath)
            except OSError:
                continue

            # Only process if file changed since last read
            if safe_name in self.cam_pubs:
                pub, last_mtime = self.cam_pubs[safe_name]
                if mtime <= last_mtime:
                    continue
            else:
                # Create publisher for this topic
                # Convert safe_name back to topic: spot_camera_back_fisheye_image_raw -> /spot/camera/back_fisheye/image_raw
                topic = '/' + safe_name.replace('_', '/')
                # Fix common patterns: restore underscores in known segments
                topic = self._restore_topic_name(safe_name)
                pub = self.create_publisher(Image, topic, 1)
                self.get_logger().info(f'Publishing relay for {topic}')
                last_mtime = 0.0
                self.cam_pubs[safe_name] = (pub, last_mtime)

            try:
                with open(filepath, 'rb') as f:
                    data = f.read()

                if len(data) < 16:
                    continue

                # Parse header
                width, height, step, enc_len = struct.unpack('<IIII', data[:16])
                encoding = data[16:16 + enc_len].decode('utf-8')
                image_data = data[16 + enc_len:]

                # Build and publish Image message
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.height = height
                msg.width = width
                msg.encoding = encoding
                msg.is_bigendian = 0
                msg.step = step
                msg.data = image_data

                pub.publish(msg)
                self.cam_pubs[safe_name] = (pub, mtime)

            except (OSError, struct.error):
                continue

    def _restore_topic_name(self, safe_name):
        """Restore original topic name from safe filename.

        The DINO node replaces '/' with '_' and strips the leading '_'.
        We need to restore the original topic structure.

        e.g. 'spot_camera_back_fisheye_image_raw' -> '/spot/camera/back_fisheye/image_raw'

        We use the known Gazebo camera topic pattern to restore correctly.
        """
        # Known camera topic patterns from Gazebo spot sim
        known_patterns = {
            'spot_camera_back_fisheye_image_raw': '/spot/camera/back_fisheye/image_raw',
            'spot_camera_frontleft_fisheye_image_raw': '/spot/camera/frontleft_fisheye/image_raw',
            'spot_camera_frontright_fisheye_image_raw': '/spot/camera/frontright_fisheye/image_raw',
            'spot_camera_thermal_camera': '/spot/camera/thermal_camera',
            'spot_camera_thermal_rgb': '/spot/camera/thermal_rgb',
        }

        if safe_name in known_patterns:
            return known_patterns[safe_name]

        # Fallback: write a topic_names.json from the DINO node side
        names_file = os.path.join(CAM_RELAY_DIR, 'topic_names.json')
        if os.path.exists(names_file):
            try:
                import json
                with open(names_file, 'r') as f:
                    mapping = json.load(f)
                if safe_name in mapping:
                    return mapping[safe_name]
            except Exception:
                pass

        # Last resort: replace _ with / and prepend /
        return '/' + safe_name.replace('_', '/')


def main():
    rclpy.init()
    node = CameraRelayPub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down camera relay publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
