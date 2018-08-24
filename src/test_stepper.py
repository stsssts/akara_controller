#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from akara_msgs.msg import Thruster
from akara_msgs.srv import BCS

MANIPULATOR_STEPS = 50

MOVE_TO_POSITIVE = 4
MOVE_TO_NEUTRAL = 5
MOVE_TO_NEGATIVE = 6
STOP = 1
SPEED_STEPS = 5

class RovController:
    def __init__ (self):
        rospy.init_node('surface_side', anonymous=True)
        rospy.logwarn("!")
        self.data = None
        self.speed = 0

        rospy.Subscriber("joy", Joy, self.callback)
        self.pub_manipulator = rospy.Publisher("buoyancy", Thruster, queue_size=1)
        self.call_move = rospy.ServiceProxy('bcs_service', BCS)

    def parse_buttons(self, message):
        if message.axes[5]:
            out = Thruster()
            out.power = [-1*message.axes[5]*MANIPULATOR_STEPS]
            self.pub_manipulator.publish(out)
        if message.axes[6]:
            self.speed += message.axes[6] * SPEED_STEPS
            if self.speed < 0:
                self.speed = 0
            elif self.speed > 255:
                self.speed = 255
            self.call_move(str(self.speed))
            rospy.loginfo('speed = %i', self.speed)
            
        if message.buttons[MOVE_TO_POSITIVE]:
            self.call_move('positive')
        elif message.buttons[MOVE_TO_NEUTRAL]:
            self.call_move('neutral')
        elif message.buttons[MOVE_TO_NEGATIVE]:
            self.call_move('negative')
        elif message.buttons[STOP]:
            self.call_move('stop')

    def callback(self, data):
        self.data = data

    def apply_data(self):
        self.parse_buttons(self.data)

    def loop(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            if self.data is not None:
                self.apply_data()
            rate.sleep()

if __name__ == '__main__':
    core = RovController()
    core.loop()
