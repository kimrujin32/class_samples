from tkinter import Tk, Label
from PIL import Image, ImageTk

# Initialize the Tkinter window
window = Tk()
window.title("Display Pizza Image")
window.geometry("400x400")  # Set window size

# Load and display the pizza.png image
image_path = "pizza.png"  # Ensure pizza.png is in the same directory as the script
image = Image.open(image_path)
photo = ImageTk.PhotoImage(image)

# Create a Label widget to display the image
label = Label(window, image=photo)
label.image = photo  # Keep a reference to avoid garbage collection
label.pack()

# Run the Tkinter event loop
window.mainloop()
