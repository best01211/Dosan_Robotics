#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray, Twist
from srv_call_test import TurtlebotArmClient
import time
import getkey
from sensor_msgs.msg import JointState

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
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)
        
        self.yolofind = False
        # yolo로 찾았다면 True yolo로 물체를 포착하지 않았으면 False
        self.armrun = False
        # 로봇암이 움직이면 True 멈춰있으면 False
        self.yolo_x = 0
        self.yolo_y = 0
        
        self.twist = Twist()
        self.data_list = []                # data_list 초기화        

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.subscription                  # prevent unused variable warning

        self.get_joint = False

    def joint_states_callback(self, msg):
        if self.get_joint == False:
            return
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint1' in name:
                print(f'joint1 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint2' in name:
                print(f'joint2 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint3' in name:
                print(f'joint3 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint4' in name:
                print(f'joint4 : {position}')

    def listener_callback(self,msg):
        if not self.armrun:               # 로봇 암이 동작 중이 아니면
            self.data = msg.data
            try:
                self.data_list = ast.literal_eval(self.data)
                print(self.data_list)

                # # 데이터가 예상한 구조인지 확인
                # if len(self.data_list) > 0 :
                #     for item in self.data_list:
                #         if item[0] == 2:  
                #             self.yolo_x = item[1]
                #             self.yolo_y = item[2]             
                #             break  
                #     print(f"Detected coordinates: {self.yolo_x}, {self.yolo_y}")
                #     print("done")
                #     if not self.yolofind and abs(self.yolo_x) < 0.01:
                #         self.twist.linear.x = 0.0
                #         self.twist.angular.z = 0.0
                #         self.cmd_vel_publisher.publish(self.twist)  
                #         self.yolofind = True
                #         self.arm_controll()
                #     elif not self.yolofind and self.yolo_x > 0.01:
                #         self.twist.linear.x = - 0.01
                #         self.twist.angular.z = 0.0
                #         self.cmd_vel_publisher.publish(self.twist)  
                #     elif not self.yolofind and self.yolo_x < -0.01:
                #         self.twist.linear.x = 0.01
                #         self.twist.angular.z = 0.0
                #         self.cmd_vel_publisher.publish(self.twist)      
                # else:
                #     self.get_logger().warn("Invalid data format: data_list is empty or does not have the expected structure.")
            
            except Exception as e:
                self.get_logger().error(f"Error processing the data: {e}")

    def append_pose_init(self, x,y,z):
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

            response = arm_client.send_request(1, "box_home_01")
            arm_client.get_logger().info(f'Response: {response.response}')            
            
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            pose_array = self.append_pose_init(0.0103589 ,-0.2600000  ,0.205779  - self.yolo_y + 0.07 )

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

            pose_array = self.append_pose_init(0.0103589,-0.2900000   ,0.205779  - self.yolo_y + 0.07 )

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')
            
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(1, "box_up_01")
            arm_client.get_logger().info(f'Response: {response.response}')    
            
            response = arm_client.send_request(1, "box_home_01")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)
            print("jobs_done")

            self.armrun = False
            self.yolofind = False  # 작업 완료 후 초기화

def main(args=None):
    rclpy.init(args=args)
    print("start")
    node = YoloDetect()
    print("YoloDetect init done")
    #rclpy.spin(node)

    arm_client = TurtlebotArmClient()
    print("TurtlebotArmClient init done")

    response = arm_client.send_request(2, "open")
    arm_client.get_logger().info(f'Response: {response.response}')

    response = arm_client.send_request(1, "box_home_01")
    arm_client.get_logger().info(f'Response: {response.response}')

    box_y_offset =  0.005
    box_z_offset =  0.045
    print("init done")

    while(rclpy.ok()):
        key_value = getkey.getkey()
        if key_value == '1':                     # open and box_home_01
            print(f"No. {key_value}")
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(1, "box_home_01")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == '2':                   # go to box first step
            print(f"No. {key_value}")
            # pose_array = node.append_pose_init(0.0103589 ,-0.2600000  ,0.205779  - node.yolo_y + 0.07)
            pose_array = node.append_pose_init(0.01 ,-0.2400000 + box_y_offset, 0.2 + box_z_offset)

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == '3':                   # go to box second step
            print(f"No. {key_value}")
            # pose_array = node.append_pose_init(0.0103589,-0.2900000   ,0.205779  - node.yolo_y + 0.07)
            pose_array = node.append_pose_init(0.01, -0.2900000 + box_y_offset, 0.2 + box_z_offset)

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == '4':                   # close
            print(f"No. {key_value}")
            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == '5':                   # close
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            
        elif key_value == '6':                    # lift up box(box_up_01)
            print(f"No. {key_value}")
            # pose_array = node.append_pose_init(0.0103589,-0.2900000   ,0.205779  - node.yolo_y + 0.07 )
            pose_array = node.append_pose_init(0.01, -0.2900000 + box_y_offset, 0.2 + 0.05 + box_z_offset)

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

        elif key_value == '7':
            print(f"No. {key_value}")
            response = arm_client.send_request(1, "box_home_01")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)
            
        elif key_value == '8':
            print(f"No. {key_value}")
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '9':
            print(f"No. {key_value}")
            print(f'box_y_offset [{box_y_offset}] box_z_offset[{box_z_offset}]')

        elif key_value == 'w':
            print(f"No. {key_value}")
            box_y_offset += 0.005
            # pose_array = node.append_pose_init(0.0103589,-0.2900000   ,0.205779  - node.yolo_y + 0.07 )
            pose_array = node.append_pose_init(0.01, -0.2900000 + box_y_offset, 0.205779 + box_z_offset)

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

        elif key_value == 's':
            print(f"No. {key_value}")
            box_y_offset -= 0.005
            # pose_array = node.append_pose_init(0.0103589,-0.2900000   ,0.205779  - node.yolo_y + 0.07 )
            pose_array = node.append_pose_init(0.01, -0.2900000 + box_y_offset, 0.205779 + box_z_offset)

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

        elif key_value == 'e':
            print(f"No. {key_value}")
            box_z_offset += 0.005
            # pose_array = node.append_pose_init(0.0103589,-0.2900000   ,0.205779  - node.yolo_y + 0.07 )
            pose_array = node.append_pose_init(0.01, -0.2900000 + box_y_offset, 0.205779 + box_z_offset)

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

        elif key_value == 'd':
            print(f"No. {key_value}")
            box_z_offset -= 0.005
            # pose_array = node.append_pose_init(0.0103589,-0.2900000   ,0.205779  - node.yolo_y + 0.07 )
            pose_array = node.append_pose_init(0.01, -0.2900000 + box_y_offset, 0.205779 + box_z_offset)

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

        elif key_value == 'q':
            print(f"No. {key_value}")
            break

        elif key_value == 'A':
            print(f"No. {key_value}")
            while 1:
                rclpy.spin_once(node)
                print("spin_once")

                # 데이터가 예상한 구조인지 확인
                if len(node.data_list) > 0 :
                    for item in node.data_list:
                        if item[0] == 2:  
                            node.yolo_x = item[1]
                            node.yolo_y = item[2]             
                            break  
                    print(f"Detected coordinates: {node.yolo_x}, {node.yolo_y}")
                    print("done")

                    if not node.yolofind and abs(node.yolo_x) < 0.01:
                        node.twist.linear.x = 0.0
                        node.twist.angular.z = 0.0
                        node.cmd_vel_publisher.publish(node.twist)  
                        #node.yolofind = True
                        #node.arm_controll()
                        print('center')
                        break
                    elif not node.yolofind and node.yolo_x > 0.01:
                        node.twist.linear.x = - 0.01
                        node.twist.angular.z = 0.0
                        node.cmd_vel_publisher.publish(node.twist)  
                        print('left')
                    elif not node.yolofind and node.yolo_x < -0.01:
                        node.twist.linear.x = 0.01
                        node.twist.angular.z = 0.0
                        node.cmd_vel_publisher.publish(node.twist)      
                        print('right')
                    else:
                        print('None')
                else:
                    node.get_logger().warn("Invalid data format: data_list is empty or does not have the expected structure.")

        elif key_value == '?':        # get joint value
            print(f"No. {key_value}")
            node.get_joint = True
            rclpy.spin_once(node)
            node.get_joint = False


        elif key_value == '>':
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '<':
            print(f"No. {key_value}")
            node.twist.linear.x = -0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        # elif key_value == '6':
        #     print(f"No. {key_value}")
        #     node.twist.linear.x = 0.5
        #     node.twist.angular.z = 0.1
        #     node.cmd_vel_publisher.publish(node.twist)            

        # elif key_value == '7':
        #     print(f"No. {key_value}")
        #     node.twist.linear.x = 0.5
        #     node.twist.angular.z = -0.1
        #     node.cmd_vel_publisher.publish(node.twist)            

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
