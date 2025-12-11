#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QLineEdit, QPushButton, QLabel)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PublisherWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ros()
        self.init_ui()

    def init_ros(self):
        # Initialize ROS2 node and publisher
        rclpy.init(args=None)
        
        # . Reliable QoS Profile
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # . Transient Local Durability Profile
        transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # . Best Effort QoS Profile
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        
        self.node = Node('pyqt_publisher')
        self.publisher = self.node.create_publisher(String, 'message_topic', 10)
        
        # Create timer for ROS2 callbacks
        self.timer = self.node.create_timer(0.1, self.timer_callback)

    def init_ui(self):
        # Set window properties
        self.setWindowTitle('Message Publisher')
        self.setGeometry(100, 100, 400, 200)

        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Create status label
        self.status_label = QLabel('Ready to publish')
        layout.addWidget(self.status_label)

        # Create input field
        self.input_field = QLineEdit()
        self.input_field.setPlaceholderText('Enter your message here')
        layout.addWidget(self.input_field)

        # Create send button
        self.send_button = QPushButton('Send Message')
        self.send_button.clicked.connect(self.publish_message)
        layout.addWidget(self.send_button)

    def publish_message(self):
        if self.input_field.text():
            # Create and publish message
            msg = String()
            msg.data = self.input_field.text()
            self.publisher.publish(msg)
            
            # Update status and clear input
            self.status_label.setText(f'Published: {msg.data}')
            self.input_field.clear()
        else:
            self.status_label.setText('Please enter a message')

    def timer_callback(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def closeEvent(self, event):
        # Clean up ROS2 nodes
        self.node.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)

def main():
    app = QApplication(sys.argv)
    window = PublisherWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()