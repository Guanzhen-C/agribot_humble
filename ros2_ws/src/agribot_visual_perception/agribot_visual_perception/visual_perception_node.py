"""ROS 2 node exposing AI vision only for observation and analysis."""

import json
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .algorithms import SUPPORTED_MODES, VisualProcessor


class VisualPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("visual_perception")
        mode = str(self.declare_parameter("mode", "object_detection").value)
        if mode not in SUPPORTED_MODES:
            raise ValueError(
                f"mode must be one of {', '.join(SUPPORTED_MODES)}; got {mode}"
            )
        input_topic = str(
            self.declare_parameter("input_topic", "/camera/rgb/image_raw").value
        )
        output_topic = str(
            self.declare_parameter(
                "output_topic", "/vision/annotated_image"
            ).value
        )
        status_topic = str(
            self.declare_parameter("status_topic", "/vision/status").value
        )
        result_topic = str(
            self.declare_parameter("result_topic", "/vision/results").value
        )
        model_dir = str(self.declare_parameter("model_dir", "").value)
        model_path = str(self.declare_parameter("model_path", "").value)
        device = str(self.declare_parameter("device", "").value)
        confidence = float(self.declare_parameter("confidence", 0.35).value)
        image_size = int(self.declare_parameter("image_size", 640).value)
        self.max_rate_hz = max(
            0.1, float(self.declare_parameter("max_rate_hz", 10.0).value)
        )

        self.mode = mode
        self.processor = VisualProcessor(
            mode,
            model_dir=model_dir,
            model_path=model_path,
            device=device,
            confidence=confidence,
            image_size=image_size,
        )
        self.bridge = CvBridge()
        self.received_frames = 0
        self.processed_frames = 0
        self.dropped_frames = 0
        self.last_metric_name = "none"
        self.last_metric_value = 0.0
        self.last_processing_ms = 0.0
        self.last_processed_at = 0.0
        self.started_at = time.monotonic()

        self.image_publisher = self.create_publisher(Image, output_topic, 2)
        self.result_publisher = self.create_publisher(String, result_topic, 2)
        self.status_publisher = self.create_publisher(String, status_topic, 2)
        self.subscription = self.create_subscription(
            Image, input_topic, self._on_image, qos_profile_sensor_data
        )
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"Visual perception ready: mode={mode}, input={input_topic}, "
            f"output={output_topic}, results={result_topic}; "
            "navigation outputs are not published"
        )

    def _on_image(self, message: Image) -> None:
        self.received_frames += 1
        now = time.monotonic()
        if now - self.last_processed_at < 1.0 / self.max_rate_hz:
            self.dropped_frames += 1
            return
        started = time.perf_counter()
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            result = self.processor.process(image)
            output = self.bridge.cv2_to_imgmsg(result.image, encoding="bgr8")
        except Exception as exc:
            self.dropped_frames += 1
            self.get_logger().error(f"Visual frame processing failed: {exc}")
            return

        output.header = message.header
        self.image_publisher.publish(output)
        result_message = String()
        result_message.data = json.dumps(
            {
                "mode": self.mode,
                "stamp": {
                    "sec": message.header.stamp.sec,
                    "nanosec": message.header.stamp.nanosec,
                },
                "frame_id": message.header.frame_id,
                "objects": result.objects,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.result_publisher.publish(result_message)
        self.processed_frames += 1
        self.last_processed_at = now
        self.last_metric_name = result.metric_name
        self.last_metric_value = result.metric_value
        self.last_processing_ms = (time.perf_counter() - started) * 1000.0

    def _publish_status(self) -> None:
        elapsed = max(time.monotonic() - self.started_at, 1e-6)
        status = {
            "mode": self.mode,
            "received_frames": self.received_frames,
            "processed_frames": self.processed_frames,
            "dropped_frames": self.dropped_frames,
            "average_processed_hz": self.processed_frames / elapsed,
            "last_processing_ms": self.last_processing_ms,
            self.last_metric_name: self.last_metric_value,
            "motion_control_connected": False,
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
