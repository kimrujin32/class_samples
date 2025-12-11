import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from move_base_msgs.action import MoveBase
from geometry_msgs.msg import PoseStamped
from my_turtlebot3_actions.action import MoveToTarget
import time

class TurtleBot3DeliveryActionServer(Node):
    def __init__(self):
        super().__init__('turtlebot3_delivery_action_server')
        self._action_server = ActionServer(
            self,
            MoveToTarget,
            'move_to_target',
            self.execute_callback
        )
        self.initial_position = None
        self.get_logger().info("Action server is running...")

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal: x={goal_handle.request.x}, y={goal_handle.request.y}, theta={goal_handle.request.theta}")
        
        # Store the initial position (Assuming the robot is at the initial position when the action starts)
        self.initial_position = (0.0, 0.0, 0.0)  # Update this logic as per your setup

        # Step 1: Move to target position
        self.feedback_callback("Moving to target...")
        result = self.move_to_target(goal_handle.request.x, goal_handle.request.y, goal_handle.request.theta)

        if not result:
            self.feedback_callback("Failed to reach target.")
            goal_handle.abort()
            return MoveToTarget.Result(success=False)

        # Step 2: Wait at the target for 10 seconds
        self.feedback_callback("Waiting at target for 10 seconds...")
        time.sleep(10)

        # Step 3: Return to initial position
        self.feedback_callback("Returning to initial position...")
        self.move_to_target(self.initial_position[0], self.initial_position[1], self.initial_position[2])

        self.feedback_callback("Delivery completed successfully.")
        goal_handle.succeed()
        return MoveToTarget.Result(success=True)

    def feedback_callback(self, message):
        self.get_logger().info(message)

    def move_to_target(self, x, y, theta):
        # Simulate the movement using move_base (adjust this to integrate with your navigation stack)
        self.get_logger().info(f"Moving to {x}, {y}, {theta}")
        # You can create a goal message to send to move_base action server
        move_goal = MoveBase.Goal()
        move_goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.w = 1.0  # Update with theta if needed
        
        # You would normally send this goal to the `move_base` server
        return True  # Simulated success, replace with actual navigation logic (e.g., move_base)

def main():
    rclpy.init()
    action_server = TurtleBot3DeliveryActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
