# import pygetwindow as gw
import pyautogui

def mouse_relative_position_from_center_normalized():
    screen_width, screen_height = pyautogui.size()
    center_x, center_y = screen_width / 2, screen_height / 2
    mouse_x, mouse_y = pyautogui.position()

    relative_x = (mouse_x - center_x) / (center_x)
    relative_y = (mouse_y - center_y) / (center_y)

    return relative_x, relative_y

def on_press(key):
    global target_roll
    global target_pitch
    if key.char == "w":
        print("w pressed")
        target_pitch = -10
    elif key.char == "a":
        print("a pressed")
        target_roll = -10
    elif key.char == "s":
        print("s pressed")
        target_pitch = 10
    elif key.char == "d":
        print("d pressed")
        target_roll = 10