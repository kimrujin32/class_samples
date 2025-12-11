import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QDoubleSpinBox, QPushButton, QMessageBox)
from PyQt5.QtCore import Qt

class InitPoseServiceNode(Node):
    def __init__(self):
        super().__init__('initpose_service_node')
        
        # Create a publisher for initial pose
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        self.get_logger().info('InitPose Service Node has been started.')

    def publish_initial_pose(self, x, y, theta):
        """
        Publish the initial pose to the /initialpose topic
        
        :param x: X coordinate of initial pose
        :param y: Y coordinate of initial pose
        :param theta: Orientation (yaw) of initial pose
        """
        # Create PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped()
        
        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set pose position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Set orientation (convert theta to quaternion)
        import math
        import tf_transformations  # You'll need to install this: pip install transforms3d
        
        # Convert yaw (theta) to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0, 0, theta)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        
        # Set covariance (you might want to adjust these values)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
        ]
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published initial pose: x={x}, y={y}, theta={theta}')

class InitPoseWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()

    def initUI(self):
        self.setWindowTitle('ROS2 Initial Pose Setter')
        self.setGeometry(300, 300, 300, 250)

        layout = QVBoxLayout()

        # X coordinate input
        x_layout = QHBoxLayout()
        x_label = QLabel('X:')
        self.x_spinbox = QDoubleSpinBox()
        self.x_spinbox.setRange(-100, 100)
        self.x_spinbox.setDecimals(3)
        x_layout.addWidget(x_label)
        x_layout.addWidget(self.x_spinbox)
        layout.addLayout(x_layout)

        # Y coordinate input
        y_layout = QHBoxLayout()
        y_label = QLabel('Y:')
        self.y_spinbox = QDoubleSpinBox()
        self.y_spinbox.setRange(-100, 100)
        self.y_spinbox.setDecimals(3)
        y_layout.addWidget(y_label)
        y_layout.addWidget(self.y_spinbox)
        layout.addLayout(y_layout)

        # Theta (rotation) input
        theta_layout = QHBoxLayout()
        theta_label = QLabel('Theta (rad):')
        self.theta_spinbox = QDoubleSpinBox()
        self.theta_spinbox.setRange(-3.14, 3.14)
        self.theta_spinbox.setDecimals(3)
        theta_layout.addWidget(theta_label)
        theta_layout.addWidget(self.theta_spinbox)
        layout.addLayout(theta_layout)

        # Publish button
        publish_btn = QPushButton('Publish Initial Pose')
        publish_btn.clicked.connect(self.publish_pose)
        layout.addWidget(publish_btn)

        self.setLayout(layout)

    def publish_pose(self):
        try:
            x = self.x_spinbox.value()
            y = self.y_spinbox.value()
            theta = self.theta_spinbox.value()

            self.node.publish_initial_pose(x, y, theta)
            
            # Show success message
            QMessageBox.information(self, 'Success', 
                                    f'Initial pose published:\nX: {x}\nY: {y}\nTheta: {theta}')
        except Exception as e:
            QMessageBox.critical(self, 'Error', str(e))

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the ROS2 node
    ros2_node = InitPoseServiceNode()

    # Create Qt Application
    app = QApplication(sys.argv)
    
    # Create and show the window
    window = InitPoseWindow(ros2_node)
    window.show()

    # Spin ROS2 node in a separate thread
    import threading
    def ros2_spin():
        try:
            rclpy.spin(ros2_node)
        except Exception as e:
            print(f"ROS2 node spin error: {e}")

    spin_thread = threading.Thread(target=ros2_spin)
    spin_thread.start()

    # Run Qt application
    try:
        sys.exit(app.exec_())
    finally:
        # Clean up
        ros2_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()