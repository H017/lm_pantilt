#!/usr/bin/env python

from lm_pantilt.srv import Reset, ResetResponse, Pan, PanResponse, Tilt, TiltResponse, PanTilt, PanTiltResponse
from lm_pantilt.msg import PanTiltState

import rospy
import Adafruit_BBIO.PWM as PWM

class PTU:
    def __init__(self):
        rospy.init_node('ptu')

        self.pan_pin = rospy.get_param('~pan_pin', 'P8_13')
        self.pan_neutral_duty_cycle = rospy.get_param('~pan_neutral_duty_cycle', 7.4)
        self.pan_min_duty_cycle = rospy.get_param('~pan_min_duty_cycle', 2.7)
        self.pan_max_duty_cycle = rospy.get_param('~pan_max_duty_cycle', 12.1)
        self.pan_min_angle = rospy.get_param('~pan_min_angle', -90)
        self.pan_max_angle = rospy.get_param('~pan_max_angle', 90)

        self.tilt_pin = rospy.get_param('~tilt_pin', 'P9_14')
        self.tilt_neutral_duty_cycle = rospy.get_param('~tilt_neutral_duty_cycle', 11.03)
        self.tilt_min_duty_cycle = rospy.get_param('~tilt_min_duty_cycle', 4.6)
        self.tilt_max_duty_cycle = rospy.get_param('~tilt_max_duty_cycle', 10.9)
        self.tilt_min_angle = rospy.get_param('~tilt_min_angle', -90)
        self.tilt_max_angle = rospy.get_param('~tilt_max_angle', 30)

        self.frequency = rospy.get_param('~frequency', 50)
        self.tilt_arm_height = rospy.get_param('~tilt_arm_height', 0.03)

        self.ptu_publisher = rospy.Publisher('/ptu/state', PanTiltState, queue_size=10, latch=True)
        self.pantilt_state = PanTiltState()
        self.pantilt_state.pan = 0
        self.pantilt_state.tilt = 0
        
        rospy.Service('~reset', Reset, self.reset)
        rospy.Service('~pan', Pan, self.pan_service)
        rospy.Service('~tilt', Tilt, self.tilt_service)
        rospy.Service('~pan_tilt', PanTilt, self.pan_tilt)

        PWM.start(self.tilt_pin, self.tilt_neutral_duty_cycle, self.frequency)
        PWM.start(self.pan_pin, self.pan_neutral_duty_cycle, self.frequency)

        self.publish_ptu_state(0, 0)
        
    def reset(self, req):
        self.pan(0)
        self.tilt(0)

        self.publish_ptu_state(0, 0)
        
        return ResetResponse()

    def pan_tilt(self, req):
        self.pan(req.panAngle)
        self.tilt(req.tiltAngle)

        self.publish_ptu_state(req.panAngle, req.tiltAngle)
        
        return PanTiltResponse()

    def pan(self, angle):
        if(angle > self.pan_max_angle):
            angle = self.pan_max_angle
        elif(angle < self.pan_min_angle):
            angle = self.pan_min_angle
            
        PWM.set_duty_cycle(self.pan_pin, -angle * ((self.pan_max_duty_cycle - self.pan_min_duty_cycle) / (self.pan_max_angle - self.pan_min_angle)) + self.pan_neutral_duty_cycle)
        self.pan_angle = angle

    def pan_service(self, req):
        self.pan(req.angle)

        self.publish_ptu_state(pan=req.angle)
        
        return PanResponse()

    def tilt(self, angle):
        if(angle > self.tilt_max_angle):
            angle = self.tilt_max_angle
        elif(angle < self.tilt_min_angle):
            angle = self.tilt_min_angle

        PWM.set_duty_cycle(self.tilt_pin, angle * ((self.tilt_max_duty_cycle - self.tilt_min_duty_cycle) / (self.tilt_max_angle - self.tilt_min_angle)) + self.tilt_neutral_duty_cycle)
        self.tilt_angle = angle

    def tilt_service(self, req):
        self.tilt(req.angle)

        self.publish_ptu_state(tilt=req.angle)
        
        return TiltResponse()

    def publish_ptu_state(self, pan=None, tilt=None):
        if(pan is not None):
            self.pantilt_state.pan = pan

        if(tilt is not None):
            self.pantilt_state.tilt = tilt

        self.ptu_publisher.publish(self.pantilt_state)
        
    def stop(self):
        PWM.stop(self.pan_pin) 
        PWM.stop(self.tilt_pin)
        PWM.cleanup()
                            
if __name__ == '__main__':
    ptu = PTU()
    rospy.spin()
    ptu.stop()
    
    