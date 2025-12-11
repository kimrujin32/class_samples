import sys
import sqlite3
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QLineEdit, QPushButton, QTableWidget, 
                             QTableWidgetItem, QMessageBox, QComboBox, QDialog,
                             QFormLayout, QDialogButtonBox)

class AddRestaurantDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Add New Restaurant')
        self.setModal(True)
        
        layout = QFormLayout()
        
        # Restaurant Name
        self.name_input = QLineEdit()
        layout.addRow('Restaurant Name:', self.name_input)
        
        # Cuisine Type
        self.cuisine_input = QLineEdit()
        layout.addRow('Cuisine Type:', self.cuisine_input)
        
        # Buttons
        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)
        
        self.setLayout(layout)
    
    def get_restaurant_data(self):
        return {
            'name': self.name_input.text(),
            'cuisine': self.cuisine_input.text()
        }

class FoodManagementApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        #self.initDatabase()

    def initUI(self):
        self.setWindowTitle('Restaurant Food Management System')
        self.setGeometry(100, 100, 800, 600)

        # Main layout
        layout = QVBoxLayout()

        # Restaurant Selection
        restaurant_layout = QHBoxLayout()
        self.restaurant_label = QLabel('Select Restaurant:')
        self.restaurant_combo = QComboBox()
        self.restaurant_combo.currentIndexChanged.connect(self.load_food_items)
        
        # Add Restaurant Button
        self.add_restaurant_btn = QPushButton('Add Restaurant')
        self.add_restaurant_btn.clicked.connect(self.add_restaurant)
        
        restaurant_layout.addWidget(self.restaurant_label)
        restaurant_layout.addWidget(self.restaurant_combo)
        restaurant_layout.addWidget(self.add_restaurant_btn)
        layout.addLayout(restaurant_layout)

        # Food Item Input Section
        food_input_layout = QHBoxLayout()
        
        # Food Name
        self.name_label = QLabel('Food Name:')
        self.name_input = QLineEdit()
        food_input_layout.addWidget(self.name_label)
        food_input_layout.addWidget(self.name_input)

        # Price
        self.price_label = QLabel('Price:')
        self.price_input = QLineEdit()
        food_input_layout.addWidget(self.price_label)
        food_input_layout.addWidget(self.price_input)

        layout.addLayout(food_input_layout)

        # Buttons section
        button_layout = QHBoxLayout()
        
        # Add Food Item Button
        self.add_button = QPushButton('Add Food Item')
        self.add_button.clicked.connect(self.add_food_item)
        button_layout.addWidget(self.add_button)

        # Delete Food Item Button
        self.delete_button = QPushButton('Delete Food Item')
        self.delete_button.clicked.connect(self.delete_food_item)
        button_layout.addWidget(self.delete_button)

        layout.addLayout(button_layout)

        # Table to display food items
        self.table = QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(['ID', 'Food Name', 'Price', 'Restaurant'])
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table)

        self.setLayout(layout)

        # Populate restaurant combo
        self.populate_restaurants()

    def initDatabase(self):
        # Create database connection and tables
        try:
            self.conn = sqlite3.connect('restaurant_food_management.db')
            self.cursor = self.conn.cursor()
            
            # Create restaurants table
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS restaurants (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    cuisine TEXT
                )
            ''')
            
            # Create food items table
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS food_items (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    price REAL NOT NULL,
                    restaurant_id INTEGER,
                    FOREIGN KEY(restaurant_id) REFERENCES restaurants(id)
                )
            ''')
            
            self.conn.commit()
        except sqlite3.Error as e:
            QMessageBox.critical(self, 'Database Error', str(e))

    def populate_restaurants(self):
        try:
            # Clear existing items
            self.restaurant_combo.clear()
            
            self.conn = sqlite3.connect('restaurant_food_management.db')
            self.cursor = self.conn.cursor()

            # Fetch restaurants
            self.cursor.execute('SELECT id, name FROM restaurants')
            restaurants = self.cursor.fetchall()

            # Populate combo box
            for rest_id, name in restaurants:
                self.restaurant_combo.addItem(name, rest_id)

        except sqlite3.Error as e:
            QMessageBox.critical(self, 'Database Error', str(e))

    def add_restaurant(self):
        dialog = AddRestaurantDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            restaurant_data = dialog.get_restaurant_data()
            
            try:
                # Insert restaurant
                self.cursor.execute('''
                    INSERT INTO restaurants (name, cuisine) 
                    VALUES (?, ?)
                ''', (restaurant_data['name'], restaurant_data['cuisine']))
                self.conn.commit()

                # Repopulate restaurants
                self.populate_restaurants()
            except sqlite3.Error as e:
                QMessageBox.critical(self, 'Database Error', str(e))

    def add_food_item(self):
        # Check if a restaurant is selected
        if self.restaurant_combo.count() == 0:
            QMessageBox.warning(self, 'Selection Error', 'Please add a restaurant first')
            return

        name = self.name_input.text()
        price = self.price_input.text()

        # Validate inputs
        if not name or not price:
            QMessageBox.warning(self, 'Input Error', 'Food name and price are required')
            return

        try:
            # Get selected restaurant ID
            restaurant_id = self.restaurant_combo.currentData()

            # Insert food item
            self.cursor.execute('''
                INSERT INTO food_items (name, price, restaurant_id) 
                VALUES (?, ?, ?)
            ''', (name, float(price), restaurant_id))
            self.conn.commit()

            # Clear inputs
            self.name_input.clear()
            self.price_input.clear()

            # Reload food items
            self.load_food_items()
        except ValueError:
            QMessageBox.warning(self, 'Input Error', 'Price must be a number')
        except sqlite3.Error as e:
            QMessageBox.critical(self, 'Database Error', str(e))

    def delete_food_item(self):
        # Get selected row
        current_row = self.table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, 'Selection Error', 'Please select a food item to delete')
            return

        # Get food item ID from first column
        food_id = self.table.item(current_row, 0).text()

        try:
            # Delete food item
            self.cursor.execute('DELETE FROM food_items WHERE id = ?', (food_id,))
            self.conn.commit()

            # Reload food items
            self.load_food_items()
        except sqlite3.Error as e:
            QMessageBox.critical(self, 'Database Error', str(e))

    def load_food_items(self):
        # Check if a restaurant is selected
        if self.restaurant_combo.count() == 0:
            return

        # Clear existing table contents
        self.table.setRowCount(0)

        try:
            # Get selected restaurant ID
            restaurant_id = self.restaurant_combo.currentData()

            # Fetch food items for selected restaurant
            self.cursor.execute('''
                SELECT f.id, f.name, f.price, r.name 
                FROM food_items f
                JOIN restaurants r ON f.restaurant_id = r.id
                WHERE r.id = ?
            ''', (restaurant_id,))
            food_items = self.cursor.fetchall()

            # Populate table
            for row_num, (food_id, name, price, restaurant) in enumerate(food_items):
                self.table.insertRow(row_num)
                
                # Insert ID
                id_item = QTableWidgetItem(str(food_id))
                self.table.setItem(row_num, 0, id_item)
                
                # Insert Food Name
                name_item = QTableWidgetItem(name)
                self.table.setItem(row_num, 1, name_item)
                
                # Insert Price
                price_item = QTableWidgetItem(f'${price:.2f}')
                self.table.setItem(row_num, 2, price_item)
                
                # Insert Restaurant Name
                restaurant_item = QTableWidgetItem(restaurant)
                self.table.setItem(row_num, 3, restaurant_item)

        except sqlite3.Error as e:
            QMessageBox.critical(self, 'Database Error', str(e))

    def closeEvent(self, event):
        # Close database connection when app closes
        if hasattr(self, 'conn'):
            self.conn.close()

def main():
    app = QApplication(sys.argv)
    food_management_app = FoodManagementApp()
    food_management_app.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()