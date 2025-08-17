#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from exploration_benchmarking_msgs.msg import ClosestObstacle


class Laser_scan_check(Node):

    def __init__(self):
        super().__init__("laser_scan_check")

        # Declare parameter for distance threshold
        self.declare_parameter("distance_threshold", 1.0)

        self.sub = self.create_subscription(LaserScan, "scan", self.callback, 10)
        self.pub = self.create_publisher(ClosestObstacle, "closest_obstacle", 10)
        self.timer = self.create_timer(
            1.0, self.publish_closest_obstacle
        )  # Publish every second
        self.get_logger().info("Subscribed to scan topic")
        self.currently_triggered = False
        self.total_triggered = 0

        # Get the distance threshold from parameter
        self.distance_threshold = (
            self.get_parameter("distance_threshold").get_parameter_value().double_value
        )
        self.get_logger().info(f"Distance threshold set to: {self.distance_threshold}")

        self.min_distance = float("inf")
        self.triggered_distance = float("inf")
        self.min_angle = 0.0

    def callback(self, data: LaserScan):

        self.min_distance = min(data.ranges)  # Get the minimum distance
        self.min_angle = (
            data.angle_min + data.ranges.index(self.min_distance) * data.angle_increment
        )  # Get the angle of the closest obstacle

        if not self.currently_triggered:
            # Check if any range in the scan is below the threshold
            if self.min_distance < self.distance_threshold:
                self.currently_triggered = True
                self.total_triggered += 1
                self.triggered_distance = self.min_distance
                self.get_logger().warn(
                    f"Too close to an obstacle, current distance is {self.min_distance:.3f} at angle {self.min_angle:.3f}, total triggered: {self.total_triggered} times"
                )

        else:  # currently_triggered is True
            if (
                self.min_distance > self.distance_threshold * 1.1
            ):  # 1.1 to avoid oscillation around the threshold
                self.currently_triggered = False
                self.triggered_distance = float("inf")
                self.get_logger().warn("No longer too close to an obstacle")
            elif self.min_distance < 0.95 * self.triggered_distance:
                # We are getting closer !
                self.triggered_distance = self.min_distance
                self.get_logger().warn(
                    f"Too close to an obstacle, current distance is {self.min_distance:.3f} at angle {self.min_angle:.3f}, total triggered: {self.total_triggered} times"
                )

    def publish_closest_obstacle(self):
        """Publish current closest obstacle information"""
        msg = ClosestObstacle()
        msg.distance = self.min_distance
        msg.angle = self.min_angle
        msg.obstacle_detected = self.currently_triggered
        self.pub.publish(msg)


def main():
    rclpy.init(args=None)
    node = Laser_scan_check()

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

# TODO: Log location when obstacle is detected? Location of the robot (corrected odom) or the object (from odom and laser scan i guess)
