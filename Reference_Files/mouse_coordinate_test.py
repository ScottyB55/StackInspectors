import pyautogui

# Get the current mouse position
mouse_x, mouse_y = pyautogui.position()

# Get the center coordinates of the screen
screen_width, screen_height = pyautogui.size()
center_x, center_y = screen_width / 2, screen_height / 2

# Calculate the relative position of the mouse to the center of the screen
rel_x = mouse_x - center_x
rel_y = mouse_y - center_y

# Print the relative position
print(f"Mouse position relative to center: ({rel_x}, {rel_y})")