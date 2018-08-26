#!/usr/bin/env python

import rospy
import sys, select, termios, tty

from akara_msgs.msg import Thruster
from akara_msgs.srv import BCS

MANIPULATOR_STEPS = 500
SPEED_STEPS = 5


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def parse_buttons(speed, key):
    if key == 'k':
        speed += SPEED_STEPS
        if speed > 255:
            speed = 255
        call_move(str(speed))
        # rospy.loginfo('speed = %i', speed)
    if key == 'j':
        speed -= SPEED_STEPS
        if speed < 0:
            speed = 0
        call_move(str(speed))
        # rospy.loginfo('speed = %i', speed)
    if key == 'l':
        out = Thruster()
        out.power = [MANIPULATOR_STEPS]
        pub_manipulator.publish(out)
    if key == 'h':
        out = Thruster()
        out.power = [-MANIPULATOR_STEPS]
        pub_manipulator.publish(out)
    if key == 'a': 
        call_move('negative')
    if key == 'd':
        call_move('positive')
    if key == 's':
        call_move('neutral')
    if key == 'x':
        call_move('stop')
        
    return speed


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    speed = 0

    rospy.init_node('stepper_teleop')

    pub_manipulator = rospy.Publisher("buoyancy", Thruster, queue_size=1)
    call_move = rospy.ServiceProxy('bcs_service', BCS)

    
    try:
        while(1):
            key = getKey()
            speed = parse_buttons(speed, key)
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
