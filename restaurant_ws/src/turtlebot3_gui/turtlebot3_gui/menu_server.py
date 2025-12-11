# order_food_service.py
import rclpy
from rclpy.node import Node
from nav_interfaces.srv import OrderFood

class OrderFoodService(Node):
    def __init__(self):
        super().__init__('order_food_service')
        self.srv = self.create_service(OrderFood, 'order_food', self.handle_order_food)

    def handle_order_food(self, request, response):
        self.get_logger().info(f"Received order for items: {request.items}")
        # Example: Simulate successful order
        response.success = True
        response.message = f"Successfully ordered {len(request.items)} items."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = OrderFoodService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
