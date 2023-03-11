#!/usr/bin/env python3

"""
Created on Sat Mar 11 10:12:16 2023
Keyboard input reader with sys.stdin & observer pattern

@author: cajun
"""


import sys
import rospy
import os
import signal
import termios
import atexit
import threading
from select import select

from observer import *


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
        
        # Set custom signal for designating a key for exiting program
        signal.signal(signal.SIGUSR1, self.sigusr1_handler)
        
        self._thread = threading.Thread(target=self.get_key)
        self._thread.start()
        
    # switch to normal terminal
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
        while True:
            if self.kbhit():
                try:
                    self.getch()
                    
                    if self._ch == 'q':
                        print("Exiting keyboard reader program...")
                        # os.kill(os.getpid(), signal.SIGUSR1)
                        break
                    
                    self.notify_event_observers(self._ch, self._ch)
                except Exception as e:
                    print(e)
                
    def sigusr1_handler(self, signum, frame):
        print("q-key detected. Exiting program...")
        sys.exit(0)


if __name__ == '__main__':
    kb = KeyboardReader()
    print("Keyboard reader thread started...")
    
