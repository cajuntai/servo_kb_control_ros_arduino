#!/bin/env python3

from pynput import keyboard
import rospy

from observer import HasObservers

class pynputKeyboard(HasObservers):

    def __init__(self, file_name=__file__.split('/')[-1].replace('.py', '')):
        super().__init__()

        self._keyboard_flag = True
        self._suppress_flag = True
        self._exit_flag     = False

        self._space = keyboard.Key.space
        self._up    = keyboard.Key.up
        self._down  = keyboard.Key.down
        self._left  = keyboard.Key.left
        self._right = keyboard.Key.right
        self._enter = keyboard.Key.enter

    # Checks for conditions before calling the function
    def check_key():  # Decorator
        def decorator(func):
            def wrapper(self, key):
                if key == keyboard.Key.esc:  # Stop listener if ESC key pressed
                    rospy.loginfo("ESC key pressed. Stopping keyboard monitoring...")
                    self._exit_flag = True
                    return False
                if func.__name__ == "on_press" and hasattr(key, "char"):
                    if key.char == "p":
                        self._keyboard_flag = not self._keyboard_flag
                        if self._keyboard_flag:
                            print("Keyboard monitoring resumed.")
                            self._suppress_flag = True
                            return False
                        else:
                            print("Keyboard monitoring paused.")
                            self._suppress_flag = False
                            return False
                if self._keyboard_flag:
                    return func(self, key)
            return wrapper
        return decorator
    
    @check_key()
    def on_press(self, key):
        try:
            print(f'{key.char} pressed')
            self.notify_event_observers(key.char, "pressed")
        except AttributeError:
            print(f'{key} pressed')
            self.notify_event_observers(key, "pressed")

    @check_key()
    def on_release(self, key):
        try:
            print(f'{key.char} released')
            self.notify_event_observers(key.char, "released")
        except AttributeError:
            print(f'{key} released')

    def start_keyboard_monitoring(self):
        rospy.loginfo("Starting keyboard monitoring...")
        print("Press the ESC key to stop monitoring")

        # continue running this program.
        while not self._exit_flag:
            with keyboard.Listener(on_press=self.on_press, on_release=self.on_release, suppress=self._suppress_flag) as self._listener:
                self._listener.join()


if __name__ == '__main__':
    try:
        keyboard = pynputKeyboard()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"Exception in {__file__}:", e)

    