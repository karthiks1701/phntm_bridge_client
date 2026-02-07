#!/usr/bin/env python3
"""
Grounding DINO detection node for Phantom Bridge.
Subscribes to camera image topics (via CycloneDDS for Gazebo access),
reads detection prompts from file IPC, runs zero-shot object detection,
and writes annotated results to file IPC.

File IPC paths:
  /tmp/dino_prompts.json  - prompts written by chat_interface, read by this node
  /tmp/dino_results/      - detection JSONs written by this node, read by chat_interface
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

import json
import base64
import re
import os
import time
import tempfile
import numpy as np
import cv2

import torch
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
from PIL import Image as PILImage


PROMPTS_FILE = '/tmp/dino_prompts.json'
RESULTS_DIR = '/tmp/dino_results'
CAM_RELAY_DIR = '/dev/shm/cam_relay'


class GroundingDinoNode(Node):
    def __init__(self):
        super().__init__('grounding_dino')

        self.declare_parameter('detection_confidence_threshold', 0.3)
        self.declare_parameter('detection_inference_interval_sec', 2.0)
        self.declare_parameter('detection_model_name', 'IDEA-Research/grounding-dino-tiny')
        self.declare_parameter('camera_topic_filter', '')  # regex to include topics; empty = all

        self.confidence_threshold = self.get_parameter('detection_confidence_threshold').value
        self.inference_interval = self.get_parameter('detection_inference_interval_sec').value
        self.model_name = self.get_parameter('detection_model_name').value
        self.camera_topic_filter = self.get_parameter('camera_topic_filter').value

        self.get_logger().info(f'Grounding DINO Node starting (CycloneDDS + file IPC)...')
        self.get_logger().info(f'Model: {self.model_name}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Inference interval: {self.inference_interval}s')
        self.get_logger().info(f'Prompts file: {PROMPTS_FILE}')
        self.get_logger().info(f'Results dir: {RESULTS_DIR}')

        # Load model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self.device}')

        self.get_logger().info(f'Loading model {self.model_name}...')
        self.processor = AutoProcessor.from_pretrained(self.model_name)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_name).to(self.device)
        self.get_logger().info('Model loaded successfully')

        # Active detection prompts (read from file)
        self.active_prompts = []

        # Camera subscriptions: topic -> subscription
        self.camera_subs = {}
        # Latest frames: topic -> (cv2 image, header)
        self.latest_frames = {}

        # Exclude known non-camera topics
        self.excluded_topics = {'/spot/chat', '/spot/detection_prompts', '/detection_annotated'}

        # Ensure output directories exist
        os.makedirs(RESULTS_DIR, exist_ok=True)
        os.makedirs(CAM_RELAY_DIR, exist_ok=True)

        # Timer for topic discovery (camera topics via CycloneDDS)
        self.discovery_timer = self.create_timer(5.0, self.discover_camera_topics)

        # Timer for reading prompts from file
        self.prompts_timer = self.create_timer(1.0, self.poll_prompts_file)

        # Timer for inference
        self.inference_timer = self.create_timer(self.inference_interval, self.run_inference)

        # Result file counter for unique filenames
        self._result_counter = 0

        self.get_logger().info('Grounding DINO Node ready')

    def poll_prompts_file(self):
        """Read active prompts from the shared file (written by chat_interface)."""
        try:
            if not os.path.exists(PROMPTS_FILE):
                if self.active_prompts:
                    self.active_prompts = []
                    self.get_logger().info('Prompts file removed, cleared all prompts')
                return

            with open(PROMPTS_FILE, 'r') as f:
                new_prompts = json.load(f)

            if new_prompts != self.active_prompts:
                added = set(new_prompts) - set(self.active_prompts)
                removed = set(self.active_prompts) - set(new_prompts)
                self.active_prompts = new_prompts
                if added:
                    self.get_logger().info(f'Prompts added from file: {added} (total: {len(self.active_prompts)})')
                if removed:
                    self.get_logger().info(f'Prompts removed from file: {removed} (total: {len(self.active_prompts)})')
        except (json.JSONDecodeError, IOError) as e:
            self.get_logger().warn(f'Error reading prompts file: {e}')

    def discover_camera_topics(self):
        topic_list = self.get_topic_names_and_types()
        for topic_name, topic_types in topic_list:
            if 'sensor_msgs/msg/Image' not in topic_types:
                continue
            if topic_name in self.excluded_topics:
                continue
            if topic_name in self.camera_subs:
                continue
            if self.camera_topic_filter:
                try:
                    if not re.search(self.camera_topic_filter, topic_name):
                        continue
                except re.error:
                    pass

            self.get_logger().info(f'Subscribing to camera topic: {topic_name}')
            sub = self.create_subscription(
                Image, topic_name,
                lambda msg, t=topic_name: self.image_callback(t, msg),
                1)
            self.camera_subs[topic_name] = sub

        # Write topic name mapping for camera relay publisher
        if self.camera_subs:
            self._write_topic_names_mapping()

    def _write_topic_names_mapping(self):
        """Write safe_name -> topic_name mapping for camera_relay_pub."""
        mapping = {}
        for topic_name in self.camera_subs:
            safe_name = topic_name.replace('/', '_').lstrip('_')
            mapping[safe_name] = topic_name
        try:
            names_file = os.path.join(CAM_RELAY_DIR, 'topic_names.json')
            with open(names_file, 'w') as f:
                json.dump(mapping, f)
        except IOError:
            pass

    def image_callback(self, topic, msg):
        try:
            # Convert ROS Image to cv2 numpy array without cv_bridge
            dtype = np.uint8
            if msg.encoding in ('mono8', '8UC1'):
                channels = 1
            elif msg.encoding in ('bgr8', '8UC3'):
                channels = 3
            elif msg.encoding in ('rgb8',):
                channels = 3
            elif msg.encoding in ('bgra8', '8UC4', 'rgba8'):
                channels = 4
            elif msg.encoding in ('mono16', '16UC1'):
                channels = 1
                dtype = np.uint16
            else:
                channels = 3  # assume bgr8

            img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

            # Convert to BGR if needed
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'rgba8':
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            elif msg.encoding == 'bgra8' or msg.encoding == '8UC4':
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            elif msg.encoding in ('mono8', '8UC1'):
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif msg.encoding in ('mono16', '16UC1'):
                img = (img / 256).astype(np.uint8)
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            self.latest_frames[topic] = (img, msg.header)

            # Write raw ROS Image to shared memory for camera relay to domain 0
            self._relay_frame(topic, msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image from {topic}: {e}')

    def _relay_frame(self, topic, msg):
        """Write raw ROS Image message to /dev/shm/cam_relay/ for FastDDS relay."""
        try:
            import struct
            safe_name = topic.replace('/', '_').lstrip('_')
            tmp_path = os.path.join(CAM_RELAY_DIR, f'.{safe_name}.tmp')
            final_path = os.path.join(CAM_RELAY_DIR, f'{safe_name}.bin')

            encoding_bytes = msg.encoding.encode('utf-8')
            # Header: width(4) + height(4) + step(4) + encoding_len(4) + encoding + data
            header = struct.pack('<IIII', msg.width, msg.height, msg.step, len(encoding_bytes))
            with open(tmp_path, 'wb') as f:
                f.write(header)
                f.write(encoding_bytes)
                f.write(bytes(msg.data))
            os.replace(tmp_path, final_path)
        except Exception:
            pass  # non-critical, don't spam logs

    def run_inference(self):
        if not self.active_prompts:
            return
        if not self.latest_frames:
            return

        # Build text prompt: all active prompts joined by ". "
        text_prompt = '. '.join(self.active_prompts) + '.'

        for topic, (cv_image, header) in list(self.latest_frames.items()):
            try:
                self._process_frame(topic, cv_image, text_prompt)
            except Exception as e:
                self.get_logger().error(f'Inference error on {topic}: {e}')

    def _process_frame(self, topic, cv_image, text_prompt):
        # Convert BGR to RGB PIL image
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(rgb_image)

        # Run Grounding DINO
        inputs = self.processor(images=pil_image, text=text_prompt, return_tensors='pt').to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            threshold=self.confidence_threshold,
            text_threshold=self.confidence_threshold,
            target_sizes=[pil_image.size[::-1]]  # (height, width)
        )[0]

        boxes = results['boxes'].cpu().numpy()
        scores = results['scores'].cpu().numpy()
        labels = results['labels']

        if len(boxes) == 0:
            return

        # Draw bounding boxes on the image
        annotated = cv_image.copy()
        detections = []

        for box, score, label in zip(boxes, scores, labels):
            x1, y1, x2, y2 = box.astype(int)
            conf = float(score)
            label_text = label.strip()

            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Draw label background
            label_display = f'{label_text} {conf:.2f}'
            (tw, th), _ = cv2.getTextSize(label_display, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(annotated, (x1, y1 - th - 8), (x1 + tw + 4, y1), (0, 255, 0), -1)
            cv2.putText(annotated, label_display, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)

            detections.append({
                'label': label_text,
                'confidence': round(conf, 3),
                'bbox': [int(x1), int(y1), int(x2), int(y2)]
            })

        # Encode annotated image as JPEG base64
        _, jpeg_buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        image_b64 = base64.b64encode(jpeg_buf).decode('utf-8')

        # Build result message
        result = {
            'type': 'detection',
            'camera': topic,
            'image_base64': image_b64,
            'detections': detections
        }

        # Write result to file IPC (chat_interface will pick it up and publish on FastDDS)
        self._write_result_file(result)

        det_summary = ', '.join(f'{d["label"]}({d["confidence"]})' for d in detections)
        self.get_logger().info(f'Detection on {topic}: {det_summary}')

    def _write_result_file(self, result):
        """Atomically write a detection result to /tmp/dino_results/."""
        try:
            self._result_counter += 1
            filename = f'{time.time_ns()}_{self._result_counter}.json'
            tmp_path = os.path.join(RESULTS_DIR, f'.tmp_{filename}')
            final_path = os.path.join(RESULTS_DIR, filename)
            with open(tmp_path, 'w') as f:
                json.dump(result, f)
            os.replace(tmp_path, final_path)
        except IOError as e:
            self.get_logger().error(f'Failed to write result file: {e}')


def main():
    rclpy.init()
    node = GroundingDinoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down Grounding DINO node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
