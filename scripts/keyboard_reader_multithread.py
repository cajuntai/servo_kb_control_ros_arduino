#!/usr/bin/env python3

"""
Created on Sat Mar 11 10:12:16 2023
Keyboard input reader with sys.stdin & observer pattern

@author: cajun
"""

import sys
import signal
import termios
import atexit
import threading
from select import select

from observer import HasObservers


class KeyboardReader(HasObservers):

    def __init__(self):
        super().__init__()
        
        # save the terminal settings
        self._fd = sys.stdin.fileno()
        self._new_term = termios.tcgetattr(self._fd)
        self._old_term = termios.tcgetattr(self._fd)
    
        # new terminal setting unbuffered
        self._new_term[3] = (self._new_term[3] & ~termios.ICANON & ~termios.ECHO)
        
        atexit.register(self.set_normal_term)
        self.set_curses_term()
        
        # Set Ctrl + C signal for exiting thread
        if __name__ == "__main__":
            signal.signal(signal.SIGINT, self.sigint_handler)
        
        # self._thread = threading.Thread(target=self.get_key)
        # self._thread.daemon = True
        # self._stop_thread = threading.Event()
        
        if __name__ == "__main__":
            self._thread.start()
        
    # switch to normal terminal (used after exiting program, otherwise you won't see any input)
    def set_normal_term(self,):
        termios.tcsetattr(self._fd, termios.TCSAFLUSH, self._old_term)
    
    # switch to unbuffered terminal
    def set_curses_term(self):
        termios.tcsetattr(self._fd, termios.TCSAFLUSH, self._new_term)
    
    def putch(self):
        # sys.stdout.write(self._ch)
        print(self._ch)
    
    def getch(self):
        self._ch = sys.stdin.read(1)
    
    def getche(self):
        self.getch()
        self.putch()
    
    def kbhit(self):
        dr, dw, de = select([sys.stdin], [], [], 0)
        return dr
    
    def get_key(self):
        while not self._stop_thread.is_set():
            if self.kbhit():
                try:
                    self.getche()
                    
                    if self._ch == 'q':
                        print("q-key detected. Exiting program...")
                        self.stop_event_thread()
                        continue
                    
                    self.notify_event_observers(self._ch, self._ch)
                except Exception as e:
                    print(e)
    
    def stop_event_thread(self):
        self._stop_thread.set()
    
    def sigint_handler(self, signum, frame):
        print("SIGINT: Exiting program...")
        self.stop_event_thread()


if __name__ == '__main__':
    kb = KeyboardReader()
    print("Keyboard reader thread started...")
    
    import rospy
    rospy.init_node("keyoard_reader")
    rospy.spin()
    
