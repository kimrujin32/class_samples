# menu_board.py
import sys
from PyQt5 import QtWidgets, uic
from PyQt5.QtGui import QPixmap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_interfaces.srv import OrderFood  # You'll need to create this service interface
from ament_index_python.packages import get_package_share_directory
import os

# Sample menu data structure
MENU_ITEMS = [
    {
        "name": "Burger",
        "price": 9.99,
        "image": "burger.png",
        "category": "Main Course"
    },
    {
        "name": "Pizza",
        "price": 12.99,
        "image": "pizza.png",
        "category": "Main Course"
    },
    {
        "name": "Salad",
        "price": 7.99,
        "image": "salad.png",
        "category": "Starters"
    }
]

class MenuBoardNode(Node):
    def __init__(self):
        super().__init__('menu_board_node')
        self.order_client = self.create_client(OrderFood, 'order_food')
        while not self.order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = OrderFood.Request()

    def send_order(self, items):
        request = OrderFood.Request()
        request.items = items
        return self.order_client.call_async(request)
    

class MenuBoard(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()


        # Get the package directory using ament_index
        package_share_directory = get_package_share_directory('turtlebot3_gui')
        ui_file_path = os.path.join(package_share_directory, 'ui', 'menu_board.ui')

        # Load the UI file dynamically
        uic.loadUi(ui_file_path, self)

        # Load the UI file
        #uic.loadUi('menu_board.ui', self)
        
        # Initialize ROS2 node
        rclpy.init()
        self.node = MenuBoardNode()
        
        # Initialize cart
        self.cart = []
        self.setup_ui()
        
    def setup_ui(self):
        # Setup tabs
        self.tabWidget.setCurrentIndex(0)
        
        # Populate menu items
        self.populate_menu_list()
        
        # Connect buttons
        self.orderButton.clicked.connect(self.place_order)
        self.clearCartButton.clicked.connect(self.clear_cart)
        
    def populate_menu_list(self):
        for item in MENU_ITEMS:
            list_item = QtWidgets.QListWidgetItem()
            widget = QtWidgets.QWidget()
            
            # Create layout for each item
            layout = QtWidgets.QHBoxLayout()
            
            # Add image
            image_label = QtWidgets.QLabel()

            pixmap = QPixmap("./pizza.png")
            #pixmap = QPixmap(item['image'])

            if pixmap.isNull():
                print("Failed to load image!!")

            image_label.setPixmap(pixmap)
            #image_label.setPixmap(pixmap.scaled(100, 100))

            layout.addWidget(image_label)
            
            # Add item details
            details_layout = QtWidgets.QVBoxLayout()

            name_label = QtWidgets.QLabel(item['name'])
            price_label = QtWidgets.QLabel(f"${item['price']:.2f}")

            details_layout.addWidget(name_label)
            details_layout.addWidget(price_label)

            layout.addLayout(details_layout)
            
            # Add order button
            order_button = QtWidgets.QPushButton("Add to Cart")
            order_button.clicked.connect(lambda checked, item=item: self.add_to_cart(item))
            layout.addWidget(order_button)
            
            widget.setLayout(layout)
            list_item.setSizeHint(widget.sizeHint())
            self.menuList.addItem(list_item)
            self.menuList.setItemWidget(list_item, widget)
            
    def add_to_cart(self, item):
        self.cart.append(item)
        self.update_cart_display()
        
    def update_cart_display(self):
        self.cartList.clear()
        total = 0
        for item in self.cart:
            self.cartList.addItem(f"{item['name']} - ${item['price']:.2f}")
            total += item['price']
        self.totalLabel.setText(f"Total: ${total:.2f}")
        
    def clear_cart(self):
        self.cart.clear()
        self.update_cart_display()
        
    def place_order(self):
        if not self.cart:
            QtWidgets.QMessageBox.warning(self, "Empty Cart", "Please add items to your cart first!")
            return

        # Prepare order items
        items = [item['name'] for item in self.cart]
        
        # Send order through ROS2 service
        future = self.node.send_order(items)
        rclpy.spin_until_future_complete(self.node, future)
        
        # Handle response
        if future.result().success:
            QtWidgets.QMessageBox.information(self, "Success", "Order placed successfully!")
            self.clear_cart()
        else:
            QtWidgets.QMessageBox.warning(self, "Error", "Failed to place order. Please try again.")
            
    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MenuBoard()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
