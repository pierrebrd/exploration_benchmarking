import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

"""
This node filters out TF messages, so that only the ones that are not in the 'map' frame are published.
This is used in replay_from_bag.py
"""


class TfFilter(Node):
    def __init__(self):
        super().__init__("tf_filter")
        self.sub = self.create_subscription(
            TFMessage, "/tf_unfiltered", self.callback, 10
        )
        self.pub = self.create_publisher(TFMessage, "/tf", 10)

    def callback(self, msg):
        filtered = TFMessage()
        for t in msg.transforms:
            if not (t.header.frame_id == "map"):
                filtered.transforms.append(t)
        if filtered.transforms:
            self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = TfFilter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
