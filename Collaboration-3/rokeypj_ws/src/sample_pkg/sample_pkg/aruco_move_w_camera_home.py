#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import getkey
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')
        self.subscription = self.create_subscription(
            MarkerArray,
            'detected_markers',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)

        self.twist = Twist()
        self.finish_move = False


        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_joint = False
        self.marker = []

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

    def listener_callback(self, msg):
        self.target_marker_id = 0                      # Change this to the desired marker ID
        for self.marker in msg.markers:
            if self.marker.id == self.target_marker_id:
                # self.get_logger().debug(f'Marker ID: {marker.id}, PositionZ: {marker.pose.pose.position.z}')
                print(f'z:[{self.marker.pose.pose.position.z}] x:[{self.marker.pose.pose.position.x}] ')
                # if marker.pose.pose.position.z > 0.30:
                if self.marker.pose.pose.position.z > 0.40:
                    self.publish_cmd_vel(0.10)
                elif self.marker.pose.pose.position.z > 0.30:
                    self.publish_cmd_vel(0.06)
                elif self.marker.pose.pose.position.z > 0.20 :
                    self.publish_cmd_vel(0.04)
                else:
                    self.publish_cmd_vel(0.0)
                    self.finish_move = True
                break
            
    def publish_cmd_vel(self, linear_x):
        self.twist.linear.x = linear_x
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)            

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerListener()
    #rclpy.spin(node)

    joint_pub = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    trajectory_msg = JointTrajectory()

    trajectory_msg.header = Header()
    trajectory_msg.header.frame_id = ''
    trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    point = JointTrajectoryPoint()
    point.velocities = [0.0] * 4
    point.accelerations = [0.0] * 4
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 500

    trajectory_msg.points = [point]

    rclpy.spin_once(node)


    
    # <group_state name="home2" group="arm">
    #     <joint name="joint1" value="0"/>
    #     <joint name="joint2" value="-1.052"/>
    #     <joint name="joint3" value="1.1060001"/>
    #     <joint name="joint4" value="0.029145"/>
    # </group_state>
    point.positions = [0.0, -1.052, 1.106, 0.029]
    print("point",point.positions)
    joint_pub.publish(trajectory_msg)
    print("init done")

    while(rclpy.ok()):
        key_value = getkey.getkey()
        if key_value == '1':          # move to cube
            print(f"No. {key_value}")
            while 1:
                rclpy.spin_once(node)
                if(node.finish_move == True):
                    node.finish_move = False
                    break

        elif key_value == '2':        # cube home
            print(f"No. {key_value}")
            point.positions = [0.0, -0.058, -0.258, 1.94]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == '3':        # box home
            print(f"No. {key_value}")
            point.positions = [0.0, -1.052, 1.106, 0.029]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)
                
        elif key_value == '4':        # forward
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '5':        # backward
            print(f"No. {key_value}") 
            node.twist.linear.x =-0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '6':        # forward + left turn
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.1
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '7':        # forward + right turn
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z =-0.1
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '8':        # get joint value
            print(f"No. {key_value}")
            node.get_joint = True
            rclpy.spin_once(node)
            node.get_joint = False

        elif key_value == '9':        # distance and offset for marker
            print(f"No. {key_value}")
            rclpy.spin_once(node)

        elif key_value == 'q':
            print(f"No. {key_value}")
            break

        elif key_value == 'A':
            print(f"No. {key_value} : home2")
            # <group_state name="home2" group="arm">
            #     <joint name="joint1" value="0"/>
            #     <joint name="joint2" value="-1.052"/>
            #     <joint name="joint3" value="1.1060001"/>
            #     <joint name="joint4" value="0.029145"/>
            # </group_state>
            point.positions = [0.0, -1.052, 1.106, 0.029]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == 'B':
            print(f"No. {key_value} : conveyor_up")
            # <group_state name="conveyor_up" group="arm">
            #     <joint name="joint1" value="-1.57233"/>
            #     <joint name="joint2" value="-1.04924"/>
            #     <joint name="joint3" value="1.0998"/>
            #     <joint name="joint4" value="0.01227"/>
            # </group_state>
            point.positions = [-1.572, -1.049, 1.099, 0.012]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == 'C':
            # print(f"No. {key_value} : conveyor_down")
            print(f"No. {key_value} : camera_home")
            # <group_state name="camera_home" group="arm">
            #     <joint name="joint1" value="0"/>
            #     <joint name="joint2" value="-0.05522330836388308"/>
            #     <joint name="joint3" value="-0.4049709280018093"/>
            #     <joint name="joint4" value="1.9987769666149904"/>
            # </group_state>
            point.positions = [0.0, -0.058, -0.258, 1.94]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == 'D':
            print(f"No. {key_value} : test_conveyor")
            # <group_state name="test_conveyor" group="arm">
            #     <joint name="joint1" value="-1.57233"/>
            #     <joint name="joint2" value="0.8620972027917303"/>
            #     <joint name="joint3" value="-0.7577865092155067"/>
            #     <joint name="joint4" value="1.0246991663076084"/>
            # </group_state>
            point.positions = [-1.572, 0.862, -0.757, 1.024]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == 'E':
            print(f"No. {key_value} : box_home_01")
            # <group_state name="box_home_01" group="arm">
            #     <joint name="joint1" value="-1.583068173097"/>
            #     <joint name="joint2" value="-0.8375535101855601"/>
            #     <joint name="joint3" value="1.3698448435818775"/>
            #     <joint name="joint4" value="-0.4678641403051206"/>
            # </group_state>  
            point.positions = [-1.583, -0.837, 1.369, -0.467]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == 'F':
            print(f"No. {key_value} : box_up_01")
            # <group_state name="box_up_01" group="arm">
            #     <joint name="joint1" value="-1.583068173097"/>
            #     <joint name="joint2" value="0.415708793517008"/>
            #     <joint name="joint3" value="-0.549165122063"/>
            #     <joint name="joint4" value="-0.2607767339405"/>
            # </group_state>  
            point.positions = [-1.583, 0.415, -0.549, -0.260]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == 'G':
            print(f"No. {key_value} : box_up_02")
            # <group_state name="box_up_02" group="arm">
            #     <joint name="joint1" value="-1.583068173097"/>
            #     <joint name="joint2" value="-0.08436894333371"/>
            #     <joint name="joint3" value="-0.5276893910326605"/>
            #     <joint name="joint4" value="0.1564660403643354"/>
            # </group_state>  
            point.positions = [-1.583, -0.084, -0.527, 0.156]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == 'H':
            print(f"No. {key_value} : box_up_03")
            # <group_state name="box_up_03" group="arm">
            #     <joint name="joint1" value="-0.001533980"/>
            #     <joint name="joint2" value="-0.08436894333371"/>
            #     <joint name="joint3" value="-0.5276893910326605"/>
            #     <joint name="joint4" value="0.1564660403643354"/>
            # </group_state>  
            point.positions = [0.000, -0.084, -0.527, 0.156]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == 'I':
            print(f"No. {key_value} : box_back_01")
            # <group_state name="box_back_01" group="arm">
            #     <joint name="joint1" value="1.583068173097"/>
            #     <joint name="joint2" value="-0.14266021327336462"/>
            #     <joint name="joint3" value="-0.11658253987930872"/>
            #     <joint name="joint4" value="0.526155410244775"/>
            # </group_state>  
            point.positions = [1.583, -0.142, -0.116, 0.526]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == 'J':
            # print(f"No. {key_value} : box_front")
            print(f"No. {key_value} : box_back_put")
            # <group_state name="box_back_put" group="arm">
            #     <joint name="joint1" value="1.583068173097"/>
            #     <joint name="joint2" value="0.523599"/>
            #     <joint name="joint3" value="-0.698132"/>
            #     <joint name="joint4" value="0.785398"/>
            # </group_state>  
            point.positions = [1.583, 0.523, -0.698, 0.785]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)


    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()