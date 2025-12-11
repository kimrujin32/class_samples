import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
import threading

class SubscriberWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("ROS2 String Subscriber")
        self.setGeometry(200, 200, 400, 100)
        
        layout = QVBoxLayout()
        
        # Label to display the received message
        self.message_label = QLabel("Waiting for messages...", self)
        layout.addWidget(self.message_label)
        
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def update_message(self, msg):
        self.message_label.setText(f"Received: {msg.data}")

class SubscriberNode(Node):
    def __init__(self, subscriber_window):
        super().__init__('string_subscriber')
        self.subscription = self.create_subscription(String, 'message_topic', self.listener_callback, 10)
        self.subscriber_window = subscriber_window

    def listener_callback(self, msg):
        # Update the subscriber window with the received message
        self.subscriber_window.update_message(msg)

def main():
    rclpy.init(args=None)
    
    # Start Qt Application
    app = QApplication(sys.argv)
    
    # Create subscriber window and node
    subscriber_window = SubscriberWindow(None)
    subscriber_node = SubscriberNode(subscriber_window)
    subscriber_window.node = subscriber_node
    
    # Display the window
    subscriber_window.show()

    # Start ROS2 spinning in a separate thread
    def spin_ros():
        rclpy.spin(subscriber_node)

    ros_thread = threading.Thread(target=spin_ros)
    ros_thread.start()

    # Run the application
    app.exec_()

    # Shutdown ROS2 when PyQt5 app is closed
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
