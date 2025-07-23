#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Rujin Kim

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlebot3_autorace_msgs.msg import MovingParam
from tf_transformations import euler_from_quaternion


class ControlMoving(Node):
    def __init__(self):
        super().__init__('control_moving')

        self.sub_moving_state = self.create_subscription(
            MovingParam, '/control/moving/state', self.get_param, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.cb_odom, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_max_vel = self.create_publisher(Float64, '/control/max_vel', 10)
        self.pub_moving_complete = self.create_publisher(UInt8, '/control/moving/complete', 10)

        # Enum definitions
        self.TypeOfMoving = Enum('TypeOfMoving', 'idle left right forward backward')
        self.TypeOfState = Enum('TypeOfState', 'idle start stop finish')

        # Odometry info
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0

        # Moving parameters
        self.moving_type = UInt8()
        self.moving_angluar = 0.0
        self.moving_linear = 0.0
        self.moving_msg = MovingParam()

        # Moving flags
        self.is_step_start = False
        self.is_step_left = False
        self.is_step_right = False
        self.is_step_forward = False
        self.is_step_backward = False

        # Internal state
        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.lastError = 0.0

        # Publish default max_vel
        msg_pub_max_vel = Float64()
        msg_pub_max_vel.data = 0.05
        self.pub_max_vel.publish(msg_pub_max_vel)

        self.timer = self.create_timer(0.01, self.control_loop)

    def control_loop(self):
        if self.is_step_left:
            self.turn_left(self.moving_msg)
        elif self.is_step_right:
            self.turn_right(self.moving_msg)
        elif self.is_step_forward:
            self.go_forward(self.moving_msg)
        elif self.is_step_backward:
            self.go_backward(self.moving_msg)

    def get_param(self, msg):
        self.moving_type = msg.moving_type
        self.moving_angluar = msg.moving_value_angular
        self.moving_linear = msg.moving_value_linear
        self.get_logger().info(f"Received moving_type: {msg.moving_type}, angular: {msg.moving_value_angular}, linear: {msg.moving_value_linear}")

        if not self.is_step_start:
            if msg.moving_type == self.TypeOfMoving.left.value:
                self.moving_msg = msg
                self.is_step_left = True
            elif msg.moving_type == self.TypeOfMoving.right.value:
                self.moving_msg = msg
                self.is_step_right = True
            elif msg.moving_type == self.TypeOfMoving.forward.value:
                self.moving_msg = msg
                self.is_step_forward = True
            elif msg.moving_type == self.TypeOfMoving.backward.value:
                self.moving_msg = msg
                self.is_step_backward = True

    def step_completed(self):
        self.is_step_start = False
        self.is_step_left = False
        self.is_step_right = False
        self.is_step_forward = False
        self.is_step_backward = False

        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.lastError = 0.0

        self.fn_stop()

        msg = UInt8()
        msg.data = self.TypeOfState.finish.value
        self.pub_moving_complete.publish(msg)

    def turn_left(self, msg):
        if not self.is_step_start:
            self.lastError = 0.0
            self.desired_theta = self.current_theta + math.radians(msg.moving_value_angular)
            self.is_step_start = True

        error = self.fn_turn()

        if abs(error) < 0.05:
            self.step_completed()

    def turn_right(self, msg):
        if not self.is_step_start:
            self.lastError = 0.0
            self.desired_theta = self.current_theta - math.radians(msg.moving_value_angular)
            self.is_step_start = True

        error = self.fn_turn()

        if abs(error) < 0.05:
            self.step_completed()

    def go_forward(self, msg):
        if not self.is_step_start:
            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y
            self.is_step_start = True

        error = self.fn_straight(msg.moving_value_linear)

        if abs(error) < 0.005:
            self.step_completed()

    def go_backward(self, msg):
        if not self.is_step_start:
            self.lastError = 0.0
            self.start_pos_x = self.current_pos_x
            self.start_pos_y = self.current_pos_y
            self.is_step_start = True

        error = self.fn_back_straight(msg.moving_value_linear)

        if abs(error) < 0.005:
            self.step_completed()

    def fn_turn(self):
        err_theta = self.current_theta - self.desired_theta
        Kp = 0.45
        Kd = 0.03
        angular_z = Kp * err_theta + Kd * (err_theta - self.lastError)
        self.lastError = err_theta

        twist = Twist()
        twist.angular.z = -(angular_z * 2)
        self.pub_cmd_vel.publish(twist)

        return err_theta

    def fn_straight(self, desired_dist):
        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x)**2 + (self.current_pos_y - self.start_pos_y)**2) - desired_dist
        twist = Twist()
        twist.linear.x = 0.1 if err_pos < 0 else -0.1
        self.pub_cmd_vel.publish(twist)
        return err_pos

    def fn_back_straight(self, desired_dist):
        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x)**2 + (self.current_pos_y - self.start_pos_y)**2) - desired_dist
        twist = Twist()
        twist.linear.x = -0.1 if err_pos < 0 else 0.1
        self.pub_cmd_vel.publish(twist)
        return err_pos

    def fn_stop(self):
        self.pub_cmd_vel.publish(Twist())

    def cb_odom(self, msg):
        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        self.current_theta = euler_from_quaternion(quaternion)[2]

        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta += 2 * math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta -= 2 * math.pi

        self.last_current_theta = self.current_theta
        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y


def main(args=None):
    rclpy.init(args=args)
    node = ControlMoving()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.fn_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
