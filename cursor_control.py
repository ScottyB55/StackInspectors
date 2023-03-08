"""
To Run
python cursor_control.py
"""

from pynput import keyboard
import time

def on_press(key):
    try:
        print('Key {} pressed.'.format(key.char))
    except AttributeError:
        print('Key {} pressed.'.format(key))

def on_release(key):
    try:
        print('Key {} released.'.format(key.char))
    except AttributeError:
        print('Key {} released.'.format(key))

# Collect events until stopped
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    while True:
        time.sleep(1)