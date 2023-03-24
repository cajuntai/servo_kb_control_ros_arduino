#!/usr/bin/env python3

"""
Created on Sat Mar 11 11:29:39 2023

@author: Cajun Tai Ka Joon
"""

import rospy
import signal
import sys
import threading

from pynput_keyboard import pynputKeyboard
from std_msgs.msg    import Int16


class ServoPublisher:
    
    def __init__(self, servo_num=1, key=[]):
        
        self._servo_num   = servo_num
        self._key         = key if key else ["w", "s"]
        self._idx         = servo_num - 1
        self._thread      = None
        self._pulse_length = 0
        
        # queue_size=1 prevents multiple messages sent to one topic
        self._servo_angle_pub = rospy.Publisher(f"diff_angle/servo_{self._servo_num}", Int16, queue_size=1)
        
        signal.signal(signal.SIGINT, self.sigint_handler)
        
        self.assign_mode()

    def assign_mode(self):
        if strategy == "individual":
            @keyboarder.on_event(self._key)
            def execute_keyboard(_, _event_type, _event):
                def thread_publish(_, _event_type):
                    global idx, thread_flag

                    while thread_flag:
                        msg = Int16()
                        if _event_type == self._key[0]:
                            self._pulse_length = 20
                        elif _event_type == self._key[1]:
                            self._pulse_length = -20
                        msg.data = self._pulse_length
                        rospy.loginfo_throttle_identical(3, f"Publishing to: {publishers.get(idx).name}")
                        publishers.get(idx).publish(msg)
                        rospy.sleep(0.03)
                
                # print(_event_type)  # Prints out the key pressed
                """ 
                    Starts a thread to publish servo angle.
                    thread_flag used as global var to prevent multiple threads from being created 
                """
                global thread_flag
                if not thread_flag and "pressed" in _event:
                    print("inidividual")
                    thread_flag = True
                    self._thread = threading.Thread(target=thread_publish, args=(_, _event_type))
                    self._thread.start()
                elif thread_flag and "released" in _event:
                    print("inidividual released")
                    self._pulse_length = -20
                    thread_flag = False
                    self._thread = None

        elif strategy == "all":
            self._thread_flag = False

            @keyboarder.on_event(self._key)
            def execute_keyboard(_, _event_type, _event):
                def thread_publish(_, _event_type):
                    global idx
                    while self._thread_flag:
                        msg = Int16()
                        if _event_type == self._key[0]:
                            pulse_length = 20
                        elif _event_type == self._key[1]:
                            pulse_length = -20
                        msg.data = pulse_length
                        self._servo_angle_pub.publish(msg)
                        rospy.sleep(0.03)

                # print(_event_type)
                if not self._thread_flag and "pressed" in _event:
                    print("all")
                    self._thread_flag = True
                    self._thread = threading.Thread(target=thread_publish, args=(_, _event_type))
                    self._thread.start()
                elif "released" in _event:
                    print("all released")
                    self._pulse_length = -20
                    self._thread_flag = False
                    self._thread = None
                
    def sigint_handler(self, signum, frame):
        print("SIGINT: Exiting program...")
        sys.exit(0)
        

if __name__ == "__main__":
    node_name = __file__.split('/')[-1].replace('.py', '')
    rospy.init_node(node_name)
    rospy.loginfo(f"{node_name} initiated")


    # Initialize KeyboardReader class and 
    keyboarder = pynputKeyboard()
    # Register events of 0 to 9 to switch servos.
    @keyboarder.on_event([str(_) for _ in list(range(10))])
    def execute_change_servo(self, _event_type, _event):
        global strategy
        if _event != "pressed" or strategy == "all":
            return
        
        num = int(_event_type)

        global publishers, idx
        if publishers.get(num):
            idx = num
            rospy.loginfo(f"Now controlling servo_{idx}...")
        else:
            rospy.logwarn(f"servo_{num} does not exist.")

    @keyboarder.on_event([keyboarder._space])
    def switch_mode(self, _event_type, _event):
        if _event != "pressed":
            return
        
        global strategy, idx
        if strategy == "individual":
            rospy.loginfo("Switching to all mode...")
            strategy = "all"
        elif strategy == "all":
            rospy.loginfo("Switching to individual mode...")
            strategy = "individual"
            rospy.loginfo(f"Now controlling servo_{idx}...")

        for alphabet in [chr(i) for i in range(ord('a'), ord('z')+1)]:
            if keyboarder._event_observers.get(alphabet):
                del keyboarder._event_observers[alphabet]
        for k, v in globals().copy().items():
            if isinstance(v, ServoPublisher):
                print("yes")
                v.assign_mode()


    strategy = "individual"  # Only publishes servo controlled. For controlling individual servos with same key.
    # strategy = "all"         # Publishes all servos at once. For controlling servos with different keys or all at once with same keys.

    thread_flag = False  # For "inidividual" strategy
         
    servo_1 = ServoPublisher(1, ["w", "s"])
    servo_2 = ServoPublisher(3)
    servo_3 = ServoPublisher(5)
    servo_4 = ServoPublisher(7)
    
    # Adds all the publsihers of all instances of ServoPublisher.
    publishers = {}
    for k, v in vars().copy().items():
        if isinstance(v, ServoPublisher):
            publishers[v._servo_num] = v._servo_angle_pub
    
    # Pre-notifies which servo is being controlled when initializing the program.
    idx = 1
    keyboarder.notify(str(idx), idx)
    rospy.loginfo(f"Running in {strategy} mode.")
    
    # Main loop
    keyboarder.start_keyboard_monitoring()
    
    
    
    