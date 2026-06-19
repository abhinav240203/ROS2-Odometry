#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").value
        self.wheel_separation_ = self.get_parameter("wheel_separation").value

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.prev_time_ = None

        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray,
            "simple_velocity_controller/commands",
            10
        )

        self.vel_sub_ = self.create_subscription(
            TwistStamped,
            "/cmd_vel",
            self.velCallback,
            10
        )

        self.joint_sub_ = self.create_subscription(
            JointState,
            "joint_states",
            self.jointCallback,
            10
        )

        self.odom_pub_ = self.create_publisher(
            Odometry,
            "robot_controller/odom",
            10
        )

        self.speed_conversion_ = np.array([
            [self.wheel_radius_ / 2, self.wheel_radius_ / 2],
            [
                self.wheel_radius_ / self.wheel_separation_,
                -self.wheel_radius_ / self.wheel_separation_
            ]
        ])

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"

        self.br_ = TransformBroadcaster(self)

        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

    def velCallback(self, msg):

        robot_speed = np.array([
            [msg.twist.linear.x],
            [msg.twist.angular.z]
        ])

        wheel_speed = np.matmul(
            np.linalg.inv(self.speed_conversion_),
            robot_speed
        )

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [
            wheel_speed[1, 0],
            wheel_speed[0, 0]
        ]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg):

        current_time = self.get_clock().now()

        if self.prev_time_ is None:
            self.prev_time_ = current_time
            self.left_wheel_prev_pos_ = msg.position[0]
            self.right_wheel_prev_pos_ = msg.position[1]
            return

        dp_left = msg.position[0] - self.left_wheel_prev_pos_
        dp_right = msg.position[1] - self.right_wheel_prev_pos_

        dt = (current_time - self.prev_time_).nanoseconds / S_TO_NS

        if dt <= 0.0:
            return

        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]
        self.prev_time_ = current_time

        fi_left = dp_left / dt
        fi_right = dp_right / dt

        linear = (
            self.wheel_radius_ * fi_right +
            self.wheel_radius_ * fi_left
        ) / 2.0

        angular = (
            self.wheel_radius_ * fi_right -
            self.wheel_radius_ * fi_left
        ) / self.wheel_separation_

        d_s = (
            self.wheel_radius_ * dp_right +
            self.wheel_radius_ * dp_left
        ) / 2.0

        d_theta = (
            self.wheel_radius_ * dp_right -
            self.wheel_radius_ * dp_left
        ) / self.wheel_separation_

        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = quaternion_from_euler(0, 0, self.theta_)

        self.odom_msg_.header.stamp = current_time.to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.odom_pub_.publish(self.odom_msg_)

        self.transform_stamped_.header.stamp = current_time.to_msg()
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0

        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]

        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()