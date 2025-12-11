#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        
        # Create subscription
        self.subscription = self.create_subscription(
            String,
            'message_topic',
            self.callback,
            10)
        
        self.get_logger().info('Subscriber node started. Waiting for messages...')

    def callback(self, msg):
        # Print received message
        self.get_logger().info(f'Received: {msg.data}')

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create and spin the node
    node = SubscriberNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()