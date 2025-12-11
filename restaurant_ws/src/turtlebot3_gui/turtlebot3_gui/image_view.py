import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
import os
os.environ["QT_QPA_PLATFORM"] = "wayland"


class ImageViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Image Viewer")
        self.setGeometry(100, 100, 800, 600)  # x, y, width, height
        
        # Create a central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Create label to hold the image
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        
        # Load and display the image
        # Replace 'path/to/your/image.png' with your actual image path
        pixmap = QPixmap('pizza.png')
        
        # Scale pixmap to fit the window while maintaining aspect ratio
        scaled_pixmap = pixmap.scaled(700, 500, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        # Set the scaled pixmap to the label
        self.image_label.setPixmap(scaled_pixmap)
        
        # Add label to layout
        layout.addWidget(self.image_label)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ImageViewer()
    window.show()
    sys.exit(app.exec_())