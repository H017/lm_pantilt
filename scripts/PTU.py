#!/usr/bin/env python
from lm_pantilt.srv import Pan, Tilt, PanTilt

import rospy
import Adafruit_BBIO.PWM as PWM

class PTU:
    def __init__(self):
        rospy.init_node('ptu')

        self.pan_pin = rospy.get_param('~pan_pin', 'P9_13')
        self.pan_neutral_duty_cycle = rospy.get_param('~pan_neutral_duty_cycle', 7.4)
        self.pan_min_duty_cycle = rospy.get_param('~pan_min_duty_cycle', 2.7)
        self.pan_max_duty_cycle = rospy.get_param('~pan_max_duty_cycle', 12.1)

        self.tilt_pin = rospy.get_param('~tilt_pin', 'P8_14')
        # Need to test
        self.tilt_neutral_duty_cycle = rospy.get_param('~tilt_neutral_duty_cycle', 9.0)
        self.tilt_min_duty_cycle = rospy.get_param('~tilt_min_duty_cycle', 6.4)
        self.tilt_max_duty_cycle = rospy.get_param('~tilt_max_duty_cycle', 13.0)

        self.frequency = rospy.get_param('~frequency', 50)

    def reset(self, req):
        PWM.start(self.pan_pin, self.pan_neutral_duty_cycle, self.frequency)
        PWM.start(self.tilt_pin, self.tilt_neutral_duty_cycle, self.frequency)
        PWM.stop(self.pan_pin)
        PWM.stop(self.tilt_pin)
        PWM.cleanup()