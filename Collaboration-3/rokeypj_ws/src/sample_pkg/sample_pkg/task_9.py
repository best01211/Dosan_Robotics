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
        self.twist = Twist()
        
        self.yolofind = False
        # yolo로 찾았다면 True yolo로 물체를 포착하지 않았으면 False
        self.armrun = False
        # 로봇암이 움직이면 True 멈춰있으면 False
        self.yolo_x = 0
        self.yolo_y = 0
        
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

                    #print(f"Detected coordinates: {self.yolo_x}, {self.yolo_y}")
                    #print("done")

                    #if not self.yolofind:
                    #    self.yolofind = True
                    #    self.arm_controll()
                else:
                    self.get_logger().warn("Invalid data format: data_list is empty or does not have the expected structure.")
            
            except Exception as e:
                self.get_logger().error(f"Error processing the data: {e}")

    def append_pose_init(self, x,y,z):
        pose_array = PoseArray()
        pose = Pose()

        pose.position.x = x
        pose.position.y =  y
        pose.position.z =  z

        pose_array.poses.append(pose)

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

            pose_array = self.append_pose_init(0.0101294- self.yolo_x ,-0.2800000  ,0.205779  - self.yolo_y + 0.06 )

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

            pose_array = self.append_pose_init(0.0101294 - self.yolo_x,-0.3100000   ,0.205779  - self.yolo_y + 0.06 )

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            response = arm_client.send_request(1, "box_up_01")
            arm_client.get_logger().info(f'Response: {response.response}')    
            time.sleep(3)

            print("jobs_done")


            self.armrun = False
            self.yolofind = False  # 작업 완료 후 초기화

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetect()
    arm_client = TurtlebotArmClient()
    print ("task start!")

    lx = 0.14
    ly = 0.0
    lz = 0.23

    pose_array = node.append_pose_init(lx, ly, lz)
    response = arm_client.send_request(0, "", pose_array)
    arm_client.get_logger().info(f'Response: {response.response}')
    print(lx, ly, lz)    

    while(rclpy.ok()):
        key_value = getkey.getkey()
        if key_value == '1':
            rclpy.spin_once(node)
            node.armrun = False
            response = arm_client.send_request(1, "box_home_01")
            arm_client.get_logger().info(f'Response: {response.response}')
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            time.sleep(1)

        elif key_value == '2':
            rclpy.spin_once(node)
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '3':
            rclpy.spin_once(node)

            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            print(node.yolo_y, node.yolo_x)
            #####pose_array = node.append_pose_init(0.0101294- node.yolo_x ,-0.2800000  ,0.205779  - node.yolo_y + 0.06 )
            pose_array = node.append_pose_init(0.0103589 ,-0.2700000  ,0.205779  - node.yolo_y + 0.06 )

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

        elif key_value == '4':

            #####pose_array = node.append_pose_init(0.0101294 - node.yolo_x,-0.3100000   ,0.205779  - node.yolo_y + 0.06 )
            pose_array = node.append_pose_init(0.0103589,-0.3000000   ,0.205779  - node.yolo_y + 0.06 )

            response = arm_client.send_request(3, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

            response = arm_client.send_request(9, "")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)


        elif key_value == '5':

            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)
            

        elif key_value == '6':
            response = arm_client.send_request(1, "box_up_01")
            arm_client.get_logger().info(f'Response: {response.response}')    
            time.sleep(1)

        elif key_value == '7':
            pass

        elif key_value == '8':
            pass

        elif key_value == '9':
            pass



        elif key_value == 'a':
            node.start()
            time.sleep(5)
            node.stop()
            print("thread stopped")
            

        elif key_value == 'q':
            #node.destroy_node()
            #rclpy.shutdown()
            break

        elif key_value == 'a':
            node.arm_controll()



    print ("task end!")
    rclpy.shutdown()


    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # node.stop()

if __name__ == '__main__':
    main()
