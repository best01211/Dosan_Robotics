#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

# Helper function for getting a single character from keyboard
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key

class TurtleBot3ManipulationTeleop(Node):
    def __init__(self):
        super().__init__('turtlebot3_manipulation_teleop')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.positions = [0.0, 0.0, 0.0, 0.0]

        self.get_logger().info('Teleop Initialized. Press keys to control joints. Press q to quit.')
        self.print_instructions()

    def print_instructions(self):
        print("1: joint1 +, 2: joint2 +, 3: joint3 +, 4: joint4 +")
        print("q: Quit")

    def run(self):
        while rclpy.ok():
            key = get_key()

            if key == 'q':
                print("Exiting...")
                break

            try:
                idx = int(key) - 1
                if 0 <= idx < len(self.positions):
                    self.positions[idx] += 0.1
                    self.send_trajectory()
            except ValueError:
                pass

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 1
        traj.points.append(point)

        self.publisher_.publish(traj)
        self.get_logger().info(f"Sent: {self.positions}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3ManipulationTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
