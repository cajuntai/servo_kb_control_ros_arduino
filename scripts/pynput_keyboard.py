from pynput import keyboard
import rospy

from observer import HasObservers

class pynputKeyboard(HasObservers):

    def __init__(self):
        super().__init__()
        rospy.init_node(file_name)

        if __name__ == '__main__':
            self.declare_listeners()

        self._keyboard_flag = True
        self.start_keyboard_monitoring()

    def declare_listeners(self):
        @self.on_event("c")
        def on_c_press(_, _event, key):
            print("Hi")
            rospy.loginfo("Hi")
            print(self, _event, key)

    # Checks for conditions before calling the function
    def check_key():
        def decorator(func):
            def wrapper(self, key):
                if key == keyboard.Key.esc:  # Stop listener if ESC key pressed
                    rospy.loginfo("ESC key pressed. Stopping keyboard monitoring...")
                    return False
                if func.__name__ == "on_press" and hasattr(key, "char"):
                    if key.char == "p":
                        self._keyboard_flag = not self._keyboard_flag
                        if self._keyboard_flag:
                            print("Keyboard monitoring resumed.")
                        else:
                            print("Keyboard monitoring paused.")
                if self._keyboard_flag:
                    return func(self, key)
            return wrapper
        return decorator
    
    @check_key()
    def on_press(self, key):
        try:
            print(f'alphanumeric key {key.char} pressed')
            self.notify_event_observers(key.char, key.char)
        except AttributeError:
            print(f'special key {key} pressed')

    @check_key()
    def on_release(self, key):
        try:
            print(f'alphanumeric key {key.char} released')
            self.notify_event_observers(key.char, key.char)
        except AttributeError:
            print(f'special key {key} released')


    def start_keyboard_monitoring(self):
        rospy.loginfo("Starting keyboard monitoring...")
        print("Press the ESC key to stop monitoring")

        # Collect events until released
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as self._listener:
            self._listener.join()
        # ...or, in a non-blocking fashion:
        self._listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self._listener.start()


if __name__ == '__main__':
    file_name = __file__.split('/')[-1]

    try:
        keyboard = pynputKeyboard()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"Exception in {file_name}:", e)

    