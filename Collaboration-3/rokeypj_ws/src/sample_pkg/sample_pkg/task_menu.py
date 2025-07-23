#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray
from srv_call_test import TurtlebotArmClient
import time
import getkey


# ANSI 색상 코드 정의
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화


##YoloDetect
class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')        
        self.subscription = self.create_subscription(
            String,
            '/yolo/detected_info',
            self.listener_callback,
            10)
        
        self.yolofind = False
        # yolo로 찾았다면 True yolo로 물체를 포착하지 않았으면 False
        self.armrun = False
        # 로봇암이 움직이면 True 멈춰있으면 False
        self.yolo_x = 0
        self.yolo_y = 0
        
        self.count = 0
        
    def listener_callback(self,msg):
        
        if not self.armrun:  # 로봇 암이 동작 중이 아니면
        
            data = msg.data
            try:
                data_list = ast.literal_eval(data)
                print(data_list)

                # 데이터가 예상한 구조인지 확인
                if len(data_list) > 0 :
                    self.yolo_x = data_list[0][1]
                    self.yolo_y = data_list[0][2]

                    print(f"Detected coordinates: {self.yolo_x}, {self.yolo_y}")
                    print("done")

                    if not self.yolofind:
                        self.yolofind = True
                        self.arm_controll()
                        if self.count == 1:
                            rclpy.shutdown()
                else:
                    self.get_logger().warn("Invalid data format: data_list is empty or does not have the expected structure.")
            
            except Exception as e:
                self.get_logger().error(f"Error processing the data: {e}")


    def append_pose_init(self, x, y, z):
        pose_array = PoseArray()
        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose_array.poses.append(pose)

        self.get_logger().info(f"{CYAN}Pose initialized - x: {x}, y: {y}, z: {z}{RESET}")

        return pose_array
        
    def arm_controll(self):
        arm_client = TurtlebotArmClient()
        # 서비스 요청을 해서 moveit에게 움직이라고 요청함
        print ("task start!")
        print(f"Get coordinates: {self.yolo_x}, {self.yolo_y}")

        if self.yolofind:
            self.armrun = True


            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            # pose_array = self.append_pose_init(0.0505943 - self.yolo_y + 0.055, 0.0 - self.yolo_x ,0.23176 )
            # pose_array = self.append_pose_init(0.137496 + 0.055, 0.0 ,0.23176 )


            pose_array = self.append_pose_init(0.137496 - self.yolo_y + 0.055, 0.0 - self.yolo_x ,0.23176 )

            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')
            

            try:
                while(rclpy.ok()):
                    # rclpy.spin_once(node)
                    key_value = getkey.getkey()
                    
                    
                    if key_value == '1':
                        node.trajectory_msg.points[0].positions = [0.0] * 4
                        node.joint_pub.publish(node.trajectory_msg)
                        print('joint1 +')

                    elif key_value == 'y':
                        if node.trajectory_msg.points[0].positions[0] < 2.79:
                            node.trajectory_msg.points[0].positions[0] += joint_angle_delta
                            node.joint_pub.publish(node.trajectory_msg)
                            print('joint1 +')
                        else:
                            print(f'joint1 reaches upper limit: {node.trajectory_msg.points[0].positions[0]}')
                    elif key_value == 'h':
                        if node.trajectory_msg.points[0].positions[0] > -2.79:
                            node.trajectory_msg.points[0].positions[0] -= joint_angle_delta
                            node.joint_pub.publish(node.trajectory_msg)
                            print('joint1 -')
                        else:
                            print(f'joint1 reaches lower limit: {node.trajectory_msg.points[0].positions[0]}')

                    elif key_value == 'u':
                        if node.trajectory_msg.points[0].positions[1] < 1.53:
                            node.trajectory_msg.points[0].positions[1] += joint_angle_delta
                            node.joint_pub.publish(node.trajectory_msg)
                            print('joint2 +')
                        else:
                            print(f'joint2 reaches upper limit: {node.trajectory_msg.points[0].positions[1]}')
                    elif key_value == 'j':
                        if node.trajectory_msg.points[0].positions[1] > -1.74:
                            node.trajectory_msg.points[0].positions[1] -= joint_angle_delta
                            node.joint_pub.publish(node.trajectory_msg)
                            print('joint2 -')
                        else:
                            print(f'joint2 reaches lower limit: {node.trajectory_msg.points[0].positions[1]}')


                    elif key_value == 'i':
                        if node.trajectory_msg.points[0].positions[2] < 1.33:
                            node.trajectory_msg.points[0].positions[2] += joint_angle_delta
                            node.joint_pub.publish(node.trajectory_msg)
                            print('joint3 +')
                        else:
                            print(f'joint3 reaches upper limit: {node.trajectory_msg.points[0].positions[2]}')
                    elif key_value == 'k':
                        if node.trajectory_msg.points[0].positions[2] > -0.90:
                            node.trajectory_msg.points[0].positions[2] -= joint_angle_delta
                            node.joint_pub.publish(node.trajectory_msg)
                            print('joint3 -')
                        else:
                            print(f'joint3 reaches lower limit: {node.trajectory_msg.points[0].positions[2]}')

                    except Exception as e:
                        print(e)

            time.sleep(2)
            
            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')            
            

            
            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("jobs_done")


            self.armrun = False
            self.yolofind = False  # 작업 완료 후 초기화
            
            self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()