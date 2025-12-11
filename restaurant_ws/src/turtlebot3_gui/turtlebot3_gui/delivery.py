import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_turtlebot3_actions.action import MoveToTarget
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QTextEdit

class DeliveryClient(Node):
    def __init__(self):
        super().__init__('turtlebot3_delivery_client')
        self._action_client = ActionClient(self, MoveToTarget, 'move_to_target')

    def send_goal(self, x, y, theta):
        goal_msg = MoveToTarget.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.theta = theta

        self.get_logger().info(f"Sending goal to ({x}, {y}, {theta})")

        # Wait for the action server to be available
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available.")
            return

        # Send goal and handle feedback/result
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Result: {result.result.success}")

class DeliveryApp(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('TurtleBot3 Delivery Service')
        self.setGeometry(100, 100, 500, 400)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Create input fields for x, y, theta
        input_layout = QVBoxLayout()
        self.x_input = QLineEdit(self)
        self.y_input = QLineEdit(self)
        self.theta_input = QLineEdit(self)

        self.x_input.setPlaceholderText("Enter x-coordinate")
        self.y_input.setPlaceholderText("Enter y-coordinate")
        self.theta_input.setPlaceholderText("Enter theta (angle in radians)")

        input_layout.addWidget(QLabel('X Coordinate:'))
        input_layout.addWidget(self.x_input)
        input_layout.addWidget(QLabel('Y Coordinate:'))
        input_layout.addWidget(self.y_input)
        input_layout.addWidget(QLabel('Theta (angle):'))
        input_layout.addWidget(self.theta_input)

        layout.addLayout(input_layout)

        # Log Window
        self.log_window = QTextEdit(self)
        self.log_window.setReadOnly(True)
        layout.addWidget(self.log_window)

        # Start Button
        self.start_button = QPushButton('Go', self)
        self.start_button.clicked.connect(self.start_delivery)
        layout.addWidget(self.start_button)

        # Result Label
        self.result_label = QLabel(self)
        layout.addWidget(self.result_label)

        self.setLayout(layout)

        self.client = DeliveryClient()

    def start_delivery(self):
        try:
            # Get user inputs
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            theta = float(self.theta_input.text())

            # Send goal to action server
            self.client.send_goal(x, y, theta)
        except ValueError:
            self.update_log("Please enter valid numbers for x, y, and theta.")

    def update_log(self, message):
        self.log_window.append(message)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    ex = DeliveryApp()
    ex.show()
    sys.exit(app.exec_())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
