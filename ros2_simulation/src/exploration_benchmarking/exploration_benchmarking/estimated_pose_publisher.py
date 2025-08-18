#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


class EstimatedPosePublisher(Node):
    def __init__(self):
        super().__init__("estimated_pose_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "estimated_pose", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        # Get the transforms from the tf buffer
        try:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            # Translation: map->odom + odom->base_link in map frame
            # (tf2 does composition internally when requesting map->base_link)
            base_in_map = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            pose.pose.position.x = base_in_map.transform.translation.x
            pose.pose.position.y = base_in_map.transform.translation.y
            pose.pose.position.z = base_in_map.transform.translation.z
            pose.pose.orientation = base_in_map.transform.rotation

            self.publisher_.publish(pose)
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")


def main():
    rclpy.init()
    node = EstimatedPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if rclpy.ok():
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
