#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import xacro

class TurtleBot3SpawnerNode(Node):
    def __init__(self):
        super().__init__('turtlebot3_spawner')
        
        # 서비스 클라이언트 생성
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # 서비스가 사용 가능할 때까지 대기
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('서비스를 기다리는 중...')
            
        # 로봇 모델 파일 경로 설정
        self.urdf_file_path = '/home/robot/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'

        # 로봇 스폰 요청
        self.spawn_robot()

    def get_model_content(self):
        """모델 파일 내용 읽기"""
        with open(self.urdf_file_path, 'r') as file:
            model_content = file.read()
        return model_content

    def spawn_robot(self):
        """로봇 스폰 요청"""
        # 요청 객체 생성
        request = SpawnEntity.Request()
        
        # 로봇 이름 설정
        request.name = 'tb3_1'
        
        # 로봇 위치 설정
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.0
        
        # 로봇 방향 설정 (quaternion)
        request.initial_pose.orientation.x = 0.0
        request.initial_pose.orientation.y = 0.0
        request.initial_pose.orientation.z = 0.0
        request.initial_pose.orientation.w = 1.0
        
        # 모델 내용 설정
        request.xml = self.get_model_content()
        
        # 비동기로 서비스 호출
        future = self.spawn_client.call_async(request)
        
        # 콜백 함수 설정
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        """스폰 요청 결과 처리"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'로봇 "{response.status_message}" 스폰 성공!')
            else:
                self.get_logger().error(f'로봇 스폰 실패: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류 발생: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    # 스포너 노드 생성
    spawner_node = TurtleBot3SpawnerNode()
    
    try:
        # 노드 실행
        rclpy.spin(spawner_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 정리
        spawner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()