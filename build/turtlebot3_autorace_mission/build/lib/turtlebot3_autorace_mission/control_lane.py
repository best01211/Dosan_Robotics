#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, UInt8, UInt32


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_lane_state = self.create_subscription(
            UInt8,
            '/detect/lane_state',
            self.callback_lane_state,
            1
        )
        self.sub_white_pixel = self.create_subscription(
            UInt32,
            '/detect/white_pixel_count',
            self.callback_white_pixel,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        self.last_error = 0
        self.MAX_VEL = 0.1
        self.avoid_active = False
        self.avoid_twist = Twist()

        self.lane_state = 0
        self.white_pixel_count = 0
        self.rotated = False

    def callback_lane_state(self, msg):
        self.lane_state = msg.data

    def callback_white_pixel(self, msg):
        self.white_pixel_count = msg.data

    def callback_get_max_vel(self, msg):
        self.MAX_VEL = msg.data

    def callback_follow_lane(self, desired_center):
        if self.avoid_active:
            return

        twist = Twist()

        # case 1: lane_state == 3 (오른쪽 차선만 감지) → 좌회전
        if self.lane_state == 3:
            twist.linear.x = 0.03
            twist.angular.z = 1.5
            self.pub_cmd_vel.publish(twist)
            return

        # case 2: lane_state == 2 and 흰선 감지 (white_pixel_count > 8000) → 180도 회전
        if self.lane_state == 2 and self.white_pixel_count >= 8000 and not self.rotated:
            twist.linear.x = 0.0
            twist.angular.z = 1.5  # 회전 속도
            self.pub_cmd_vel.publish(twist)
            self.get_logger().info('White line detected. Rotating 180 degrees.')
            self.rotated = True
            return

        # case 3: 180도 회전 후 lane_state != 2 → 계속 직진
        if self.rotated and self.lane_state != 2:
            twist.linear.x = 0.04
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            return

        # case 4: lane_state == 1 or 2 (정상 차선 감지 후 복귀) → PD 제어 복원
        if self.lane_state in [1, 2]:
            self.rotated = False  # 상태 초기화
            center = desired_center.data
            error = center - 500
            Kp = 0.0025
            Kd = 0.007
            angular_z = Kp * error + Kd * (error - self.last_error)
            self.last_error = error

            twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
            twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            self.pub_cmd_vel.publish(twist)

    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg
        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        self.get_logger().info('Avoidance mode {}'.format('activated' if self.avoid_active else 'deactivated'))

    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
