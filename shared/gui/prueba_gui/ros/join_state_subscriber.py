from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):

    def __init__(self, callback):

        super().__init__("joint_state_subscriber")

        self.callback_gui = callback

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):

        self.callback_gui(msg.position)