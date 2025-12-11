#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import math
import time

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw)
    Roll is rotation around x-axis in radians (counterclockwise)
    Pitch is rotation around y-axis in radians (counterclockwise)
    Yaw is rotation around z-axis in radians (counterclockwise)

    Args:
        x, y, z, w: Quaternion components

    Returns:
        roll, pitch, yaw: Euler angles in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw



class Turtlebot3Node(Node):
    def __init__(self, robot_name):
        super().__init__(f'turtlebot3_{robot_name}')
        self.robot_name = robot_name
        
        # 로봇의 현재 상태
        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        # Publisher 설정
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            f'/{robot_name}/status',
            10
        )
        
        # Subscriber 설정
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{robot_name}/odom',
            self.odom_callback,
            10
        )
        
        # 다른 로봇의 상태 구독
        other_robot = 'tb3_1' if robot_name == 'tb3_0' else 'tb3_0'
        self.other_status_sub = self.create_subscription(
            String,
            f'/{other_robot}/status',
            self.other_robot_callback,
            10
        )
        
        # Timer 설정
        self.create_timer(0.1, self.motion_control)  # 10Hz
        self.create_timer(1.0, self.publish_status)  # 1Hz
        
        self.get_logger().info(f'{robot_name} 초기화 완료')

    def odom_callback(self, msg):
        """오도메트리 데이터 처리"""
        # 위치 정보 추출
        self.position['x'] = msg.pose.pose.position.x
        self.position['y'] = msg.pose.pose.position.y
        
        # 방향 정보 추출 (quaternion -> euler)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.position['theta'] = yaw

    def other_robot_callback(self, msg):
        """다른 로봇으로부터 받은 상태 메시지 처리"""
        self.get_logger().info(f'다른 로봇으로부터 받은 메시지: {msg.data}')

    def publish_status(self):
        """자신의 상태 정보를 발행"""
        msg = String()
        status_info = (f"로봇: {self.robot_name}, "
                      f"위치: ({self.position['x']:.2f}, {self.position['y']:.2f}), "
                      f"방향: {math.degrees(self.position['theta']):.2f}도")
        msg.data = status_info
        self.status_pub.publish(msg)

    def motion_control(self):
        """로봇 움직임 제어"""
        cmd_vel = Twist()
        
        # 각 로봇에 대해 다른 움직임 패턴 설정
        if self.robot_name == 'tb3_0':
            # 첫 번째 로봇: 원 운동
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.5
        else:
            # 두 번째 로봇: 8자 운동
            current_time = time.time()
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = math.sin(current_time * 0.5) * 0.5
        
        self.cmd_vel_pub.publish(cmd_vel)

def main():
    rclpy.init()
    
    # 두 개의 터틀봇3 노드 생성
    node1 = Turtlebot3Node('tb3_0')
    node2 = Turtlebot3Node('tb3_1')
    
    try:
        # 두 노드를 병렬로 실행
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node1)
        executor.add_node(node2)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node1.destroy_node()
            node2.destroy_node()
            
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

