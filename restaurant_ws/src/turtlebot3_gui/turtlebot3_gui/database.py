import sys
import pymysql
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QLineEdit, QSpinBox, QPushButton, 
                           QTableWidget, QTableWidgetItem, QComboBox, QMessageBox)
from PyQt5.QtCore import Qt

class FoodOrderSystem(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Food Order Management System")
        self.setGeometry(100, 100, 1000, 600)
        
        # Database connection
        self.db_config = {
            'host': 'localhost',
            'user': 'root',
            'password': 'love1103',
            'database': 'restaurant_db',
            'charset': 'utf8mb4'
        }
        
        # Initialize UI
        self.init_ui()
        self.load_orders()
        
    def init_ui(self):
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create input form
        form_layout = QHBoxLayout()
        
        # Table number input
        table_layout = QVBoxLayout()
        table_label = QLabel("Table Number:")
        self.table_input = QSpinBox()
        self.table_input.setRange(1, 50)
        table_layout.addWidget(table_label)
        table_layout.addWidget(self.table_input)
        form_layout.addLayout(table_layout)
        
        # Food name input
        food_layout = QVBoxLayout()
        food_label = QLabel("Food Name:")
        self.food_input = QLineEdit()
        food_layout.addWidget(food_label)
        food_layout.addWidget(self.food_input)
        form_layout.addLayout(food_layout)
        
        # Quantity input
        qty_layout = QVBoxLayout()
        qty_label = QLabel("Quantity:")
        self.qty_input = QSpinBox()
        self.qty_input.setRange(1, 100)
        qty_layout.addWidget(qty_label)
        qty_layout.addWidget(self.qty_input)
        form_layout.addLayout(qty_layout)
        
        # Price input
        price_layout = QVBoxLayout()
        price_label = QLabel("Unit Price:")
        self.price_input = QLineEdit()
        price_layout.addWidget(price_label)
        price_layout.addWidget(self.price_input)
        form_layout.addLayout(price_layout)
        
        # Status input
        status_layout = QVBoxLayout()
        status_label = QLabel("Status:")
        self.status_input = QComboBox()
        self.status_input.addItems(['pending', 'preparing', 'served', 'paid'])
        status_layout.addWidget(status_label)
        status_layout.addWidget(self.status_input)
        form_layout.addLayout(status_layout)
        
        # Buttons
        button_layout = QVBoxLayout()
        self.add_button = QPushButton("Add Order")
        self.add_button.clicked.connect(self.add_order)
        self.update_button = QPushButton("Update Order")
        self.update_button.clicked.connect(self.update_order)
        self.delete_button = QPushButton("Delete Order")
        self.delete_button.clicked.connect(self.delete_order)
        button_layout.addWidget(self.add_button)
        button_layout.addWidget(self.update_button)
        button_layout.addWidget(self.delete_button)
        form_layout.addLayout(button_layout)
        
        main_layout.addLayout(form_layout)
        
        # Create table widget
        self.table_widget = QTableWidget()
        self.table_widget.setColumnCount(8)
        self.table_widget.setHorizontalHeaderLabels([
            "Order ID", "Table", "Food Name", "Quantity", 
            "Unit Price", "Total", "Status", "Order Time"
        ])
        self.table_widget.itemClicked.connect(self.select_order)
        main_layout.addWidget(self.table_widget)
        
    def connect_db(self):
        try:
            return pymysql.connect(**self.db_config)
        except pymysql.Error as err:
            QMessageBox.critical(self, "Database Error", f"Failed to connect to database: {err}")
            return None
            
    def load_orders(self):
        conn = self.connect_db()
        if not conn:
            return
            
        with conn.cursor() as cursor:
            cursor.execute("""
                SELECT order_id, table_number, food_name, quantity, 
                       unit_price, total_amount, order_status, order_time 
                FROM food_orders 
                ORDER BY order_time DESC
            """)
            
            # Clear existing rows
            self.table_widget.setRowCount(0)
            
            # Populate table
            for row_data in cursor.fetchall():
                row = self.table_widget.rowCount()
                self.table_widget.insertRow(row)
                for col, value in enumerate(row_data):
                    item = QTableWidgetItem(str(value))
                    self.table_widget.setItem(row, col, item)
                    
        conn.close()
        
    def add_order(self):
        try:
            table_num = self.table_input.value()
            food_name = self.food_input.text()
            quantity = self.qty_input.value()
            unit_price = float(self.price_input.text())
            status = self.status_input.currentText()
            
            conn = self.connect_db()
            if not conn:
                return
                
            with conn.cursor() as cursor:
                cursor.execute("""
                    INSERT INTO food_orders 
                    (table_number, food_name, quantity, unit_price, order_status)
                    VALUES (%s, %s, %s, %s, %s)
                """, (table_num, food_name, quantity, unit_price, status))
                
            conn.commit()
            conn.close()
            
            self.load_orders()
            self.clear_inputs()
            QMessageBox.information(self, "Success", "Order added successfully!")
            
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Please enter valid numbers for price!")
        except pymysql.Error as err:
            QMessageBox.critical(self, "Database Error", f"Failed to add order: {err}")
            
    def update_order(self):
        try:
            selected_items = self.table_widget.selectedItems()
            if not selected_items:
                QMessageBox.warning(self, "Selection Error", "Please select an order to update!")
                return
                
            order_id = int(self.table_widget.item(selected_items[0].row(), 0).text())
            table_num = self.table_input.value()
            food_name = self.food_input.text()
            quantity = self.qty_input.value()
            unit_price = float(self.price_input.text())
            status = self.status_input.currentText()
            
            conn = self.connect_db()
            if not conn:
                return
                
            with conn.cursor() as cursor:
                cursor.execute("""
                    UPDATE food_orders 
                    SET table_number = %s, food_name = %s, quantity = %s, 
                        unit_price = %s, order_status = %s
                    WHERE order_id = %s
                """, (table_num, food_name, quantity, unit_price, status, order_id))
                
            conn.commit()
            conn.close()
            
            self.load_orders()
            self.clear_inputs()
            QMessageBox.information(self, "Success", "Order updated successfully!")
            
        except ValueError:
            QMessageBox.warning(self, "Input Error", "Please enter valid numbers for price!")
        except pymysql.Error as err:
            QMessageBox.critical(self, "Database Error", f"Failed to update order: {err}")
            
    def delete_order(self):
        try:
            selected_items = self.table_widget.selectedItems()
            if not selected_items:
                QMessageBox.warning(self, "Selection Error", "Please select an order to delete!")
                return
                
            order_id = int(self.table_widget.item(selected_items[0].row(), 0).text())
            
            reply = QMessageBox.question(self, "Confirm Delete",
                                       "Are you sure you want to delete this order?",
                                       QMessageBox.Yes | QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                conn = self.connect_db()
                if not conn:
                    return
                    
                with conn.cursor() as cursor:
                    cursor.execute("DELETE FROM food_orders WHERE order_id = %s", (order_id,))
                
                conn.commit()
                conn.close()
                
                self.load_orders()
                self.clear_inputs()
                QMessageBox.information(self, "Success", "Order deleted successfully!")
                
        except pymysql.Error as err:
            QMessageBox.critical(self, "Database Error", f"Failed to delete order: {err}")
            
    def select_order(self, item):
        row = item.row()
        self.table_input.setValue(int(self.table_widget.item(row, 1).text()))
        self.food_input.setText(self.table_widget.item(row, 2).text())
        self.qty_input.setValue(int(self.table_widget.item(row, 3).text()))
        self.price_input.setText(self.table_widget.item(row, 4).text())
        self.status_input.setCurrentText(self.table_widget.item(row, 6).text())
        
    def clear_inputs(self):
        self.table_input.setValue(1)
        self.food_input.clear()
        self.qty_input.setValue(1)
        self.price_input.clear()
        self.status_input.setCurrentText('pending')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = FoodOrderSystem()
    window.show()
    sys.exit(app.exec_())