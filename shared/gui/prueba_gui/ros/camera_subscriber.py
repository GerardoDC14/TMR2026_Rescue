import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraSubscriber(Node):

    def __init__(self, topic, callback):

        super().__init__("camera_subscriber_" + topic.replace("/", "_"))

        self.bridge = CvBridge()
        self.callback_gui = callback

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.callback_gui(frame)