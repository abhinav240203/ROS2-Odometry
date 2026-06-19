#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class NoisyController(Node):
    def __init__(self):
        super().__init__("noisy_controller")

        self.declare_parameter("wheel_radius", 0.035)
        self.declare_parameter("wheel_separation", 0.18)

        self.wheel_radius_ = self.get_parameter("wheel_radius").value
        self.wheel_separation_ = self.get_parameter("wheel_separation").value

        self.get_logger().info("wheel radius %f" % self.wheel_radius_)
        self.get_logger().info("wheel separation %f" % self.wheel_separation_)

        self.left_wheel_previous_position_ = 0.0
        self.right_wheel_previous_position_ = 0.0
        self.previous_time_ = None
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0



        self.joint_sub_ = self.create_subscription(
            JointState,
            "joint_states",
            self.jointCallback,
            10
        )

        self.odom_pub_ = self.create_publisher(
            Odometry, "robot_controller/odom_noisy", 10

        )

        


        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.broad_ = TransformBroadcaster(self)
        self.transform_stamped_  = TransformStamped()
        self.transform_stamped_.header.frame_id = 'odom'
        self.transform_stamped_.child_frame_id = 'base_footprint_noisy'


        



    def jointCallback(self, msg):

        wheel_encoder_left = msg.position[1] + np.random.normal(0 , 0.005)
        wheel_encoder_right = msg.position[0] + + np.random.normal(0 , 0.005)

        current_time = self.get_clock().now()

        if self.previous_time_ is None:
            self.previous_time_ = current_time
            self.left_wheel_previous_position_ = msg.position[1]
            self.right_wheel_previous_position_ = msg.position[0]
            return

        variation_left = wheel_encoder_left - self.left_wheel_previous_position_
        variation_right = wheel_encoder_right - self.right_wheel_previous_position_

        time = current_time - self.previous_time_

        dt = time.nanoseconds / S_TO_NS

        if dt == 0:
            return

        phi_left = variation_left / dt
        phi_right = variation_right / dt

        self.left_wheel_previous_position_ = msg.position[1]
        self.right_wheel_previous_position_ = msg.position[0]
        self.previous_time_ = current_time

        linear = (self.wheel_radius_ * phi_right + self.wheel_radius_ * phi_left) / 2
        angular = (self.wheel_radius_ * phi_right - self.wheel_radius_ * phi_left) / self.wheel_separation_

        d_s = (self.wheel_radius_ * variation_right + self.wheel_radius_ * variation_left)/2
        d_theta = (self.wheel_radius_ * variation_right - self.wheel_radius_ * variation_left) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        





        self.odom_pub_.publish(self.odom_msg_)
        self.broad_.sendTransform(self.transform_stamped_)



def main():
    rclpy.init()
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()                   

          




