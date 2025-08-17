#!/usr/bin/env python

# Ported to ROS2
# Due to problem with the node shutdown in ROS2, I had to create a new node to log the final distance.


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from exploration_benchmarking_msgs.msg import Distances


def euler_from_quaternion(quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    quaternion is a tuple (x, y, z, w)
    """
    x, y, z, w = quaternion

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class Distance_check(Node):

    def __init__(self):
        super().__init__("distance_check")
        self.sub = self.create_subscription(Odometry, "ground_truth", self.callback, 10)
        self.pub = self.create_publisher(Distances, "robot_distances", 10)
        self.timer = self.create_timer(
            1.0, self.publish_distances
        )  # Publish every second
        self.get_logger().info("Subscribed to ground_truth topic")
        self.total_distance = 0
        self.total_rotational_distance = 0
        self.total_manhattan_distance_x = 0
        self.total_manhattan_distance_y = 0
        self.previous_x = 0
        self.previous_y = 0
        self.previous_yaw = 0
        self.first_run = True

    def callback(self, data):
        if self.first_run:
            self.previous_x = data.pose.pose.position.x
            self.previous_y = data.pose.pose.position.y
            quaternion = (
                data.pose.pose.orientation.x,
                data.pose.pose.orientation.y,
                data.pose.pose.orientation.z,
                data.pose.pose.orientation.w,
            )
            _, _, self.previous_yaw = euler_from_quaternion(quaternion)
            self.first_run = False
            return

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        # Calculate linear distance
        d_increment = math.dist((self.previous_x, self.previous_y), (x, y))
        self.total_distance = self.total_distance + d_increment

        # Calculate Manhattan distance components
        self.total_manhattan_distance_x += abs(x - self.previous_x)
        self.total_manhattan_distance_y += abs(y - self.previous_y)

        # Calculate rotational distance
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        )
        _, _, current_yaw = euler_from_quaternion(quaternion)
        angular_diff = abs(current_yaw - self.previous_yaw)
        # Handle wrap-around
        if angular_diff > math.pi:
            angular_diff = 2 * math.pi - angular_diff

        self.total_rotational_distance += angular_diff

        # Update previous values
        self.previous_x = data.pose.pose.position.x
        self.previous_y = data.pose.pose.position.y
        self.previous_yaw = current_yaw

    def publish_distances(self):
        """Publish current distance measurements"""
        msg = Distances()
        msg.euclidean_distance = self.total_distance
        msg.manhattan_distance_x = self.total_manhattan_distance_x
        msg.manhattan_distance_y = self.total_manhattan_distance_y
        msg.rotational_distance = self.total_rotational_distance
        self.pub.publish(msg)


class Distance_check_final_logger(Node):
    def __init__(
        self,
        total_distance,
        total_rotational_distance,
        total_manhattan_distance_x,
        total_manhattan_distance_y,
        context,
    ):
        super().__init__("distance_check_final_logger", context=context)
        self.get_logger().info(f"Final distance is {total_distance}")
        self.get_logger().info(
            f"Final rotational distance is {total_rotational_distance}"
        )
        self.get_logger().info(
            f"Final Manhattan distance X is {total_manhattan_distance_x}"
        )
        self.get_logger().info(
            f"Final Manhattan distance Y is {total_manhattan_distance_y}"
        )
        # Shutdown
        self.destroy_node()


def main():
    rclpy.init(args=None)
    node = Distance_check()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"Final distance is {node.total_distance}")
        print(f"Final rotational distance is {node.total_rotational_distance}")
        print(f"Final Manhattan distance X is {node.total_manhattan_distance_x}")
        print(f"Final Manhattan distance Y is {node.total_manhattan_distance_y}")
        # We also include it in the ROS logs

        # node.get_logger().info(f"Final distance is {node.total_distance}")
        # It will probably fail, lets create a new context to log and then destroy it

        new_context = rclpy.Context()
        rclpy.init(context=new_context)
        new_node = Distance_check_final_logger(
            node.total_distance,
            node.total_rotational_distance,
            node.total_manhattan_distance_x,
            node.total_manhattan_distance_y,
            context=new_context,
        )
        rclpy.spin(new_node)

        # Cleanup
        if rclpy.ok():
            node.destroy_node()
            new_node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
