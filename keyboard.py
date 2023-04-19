import keyboard
import time

while True:
    if keyboard.read_key() == "a":
        print("You pressed 'a'.")
        break
    
    time.sleep(0.1)  # Add a small delay to control the rate of updates