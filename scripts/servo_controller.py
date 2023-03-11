#!/usr/bin/env python3

"""
Created on Sat Mar 11 11:29:39 2023

@author: cajun
"""

import rospy

from keyboard_reader import KeyboardReader
from std_msgs.msg    import Int16



class ServoPublisher:
    
    def __init__(self, servo_num="/servo_1", key=[]):
        
        self._keyboarder = KeyboardReader()
        
        self._servo_num  = servo_num
        self._key        = key if key else 'w'
        
        self._servo_angle_pub = rospy.Publisher(self._servo_num, Int16, queue_size=1)
        
        @self._keyboarder.on_event("*")
        def execute_change_servo(_, _event_type, _event):
            try:
                global publishers, idx
                _event = int(_event)
                if _event <= len(publishers) and _event > 0:
                    idx    = _event
                    rospy.loginfo(f"Now controlling servo_{idx}...")
                else:
                    rospy.logwarn(f"servo_{_event} does not exist.")
            except:
                 pass
             
        @self._keyboarder.on_event(key)
        def execute_keyboard(_, _event_type, _event):
            global idx
            msg = Int16()
            if _event == key[0]:
                pulse_length = 20
            elif _event == key[1]:
                pulse_length = -20
            msg.data = pulse_length
            publishers[idx-1].publish(msg)
            
            

if __name__ == "__main__":
    node_name = __file__.split('/')[-1].replace('.py', '')
    rospy.init_node(node_name)
    rospy.loginfo(f"{node_name} initiated")
    
    servo_1 = ServoPublisher("/servo_1", ["w", "s"])
    servo_2 = ServoPublisher("/servo_2", ["w", "s"])
    
    publishers = []
    for k, v in vars().copy().items():
        if isinstance(v, ServoPublisher):
            publishers.append(v._servo_angle_pub)
    
    idx = 1
    
    rospy.spin()
    
    
    
    
    
    
    
    
    
    
    
