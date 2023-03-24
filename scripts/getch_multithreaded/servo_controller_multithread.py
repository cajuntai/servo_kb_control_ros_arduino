#!/usr/bin/env python3

"""
Created on Sat Mar 11 11:29:39 2023

@author: cajun
"""

import rospy
import signal
import sys

from keyboard_reader import KeyboardReader
from std_msgs.msg    import Int16


class ServoPublisher:
    
    def __init__(self, servo_num=1, key=[]):
        
        # self._keyboarder = KeyboardReader()
        # self._keyboarder._thread.start()
        
        self._servo_num  = servo_num
        self._key        = key if key else ["w", "s"]
        self._idx        = servo_num - 1
        
        # queue_size=1 prevents multiple messages sent to one topic
        self._servo_angle_pub = rospy.Publisher(f"diff_angle/servo_{self._servo_num}", Int16, queue_size=1)
        
        signal.signal(signal.SIGINT, self.sigint_handler)
        
        if strategy == "individual":
            @keyboarder.on_event(self._key)
            def execute_keyboard(_, _event_type, _event):
                # print(self, self._key)
                global idx
                msg = Int16()
                if _event == self._key[0]:
                    pulse_length = 20
                elif _event == self._key[1]:
                    pulse_length = -20
                msg.data = pulse_length
                publishers[idx-1].publish(msg)
        elif strategy == "all":
            @keyboarder.on_event(self._key)
            def execute_keyboard(_, _event_type, _event):
                msg = Int16()
                if _event == self._key[0]:
                    pulse_length = 20
                elif _event == self._key[1]:
                    pulse_length = -20
                msg.data = pulse_length
                self._servo_angle_pub.publish(msg)
            
    def sigint_handler(self, signum, frame):
        print("SIGINT: Exiting program...")
        sys.exit(0)
            

""" 
NOTES:
   1. If multiple ServoPublishers are connected via the same key, this will cause all of them to publish together;
      can be ignored by seeting queue_sie to 1, but it's inefficient.'
"""

if __name__ == "__main__":
    node_name = __file__.split('/')[-1].replace('.py', '')
    rospy.init_node(node_name)
    rospy.loginfo(f"{node_name} initiated")
    
    keyboarder = KeyboardReader()
    keyboarder._thread.start()
    @keyboarder.on_event([str(_) for _ in list(range(10))])
    def execute_change_servo(self, _event_type, _event):
        try:
            _event = int(_event)  # Check if event is a number
            
            global publishers, idx
            if _event <= len(publishers) and _event > 0:
                idx = _event
                rospy.loginfo(f"Now controlling servo_{idx}...")
            else:
                rospy.logwarn(f"servo_{_event} does not exist.")
        except ValueError:
             pass
         
    strategy = "individual"  # Only executes when the servo is controlled
    strategy = "all"  # Executes regardless of which servo is being controlled
         
    servo_1 = ServoPublisher(1, ["w", "s"])
    servo_2 = ServoPublisher(2)
    servo_3 = ServoPublisher(3)
    servo_4 = ServoPublisher(4)
    
    publishers = []
    for k, v in vars().copy().items():
        if isinstance(v, ServoPublisher):
            publishers.append(v._servo_angle_pub)
    
    idx = 1
    keyboarder.notify(str(idx), idx)
    
    rospy.spin()
    
    
    
    
