import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LaneTestPublisher(Node):
    def __init__(self):
        super().__init__('lane_test_publisher')
        self.pub = self.create_publisher(String, '/detect/lane_test', 10)
        self.timer = self.create_timer(2.0, self.publish_fake_lane)

        # 테스트용 시나리오 설정
        self.states = [
            "left_yellow",
            "",
            "left_yellow",
            "horizontal_white"
        ]
        self.index = 0

    def publish_fake_lane(self):
        if self.index >= len(self.states):
            self.get_logger().info("모든 테스트 완료.")
            return
        msg = String()
        msg.data = self.states[self.index]
        self.pub.publish(msg)
        self.get_logger().info(f"테스트 상태 전송: {msg.data}")
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = LaneTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
