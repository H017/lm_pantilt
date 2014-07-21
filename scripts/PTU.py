#!/usr/bin/env python

from lm_pantilt.srv import Reset, ResetResponse, Pan, PanResponse, Tilt, TiltResponse, PanTilt, PanTiltResponse
import rospy
import tf
import math
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
        self.pan_value = 0
        self.tilt_value = 0

        self.tf_publisher = tf.TransformBroadcaster()
        
        rospy.Service('~reset', Reset, self.reset)
        rospy.Service('~pan', Pan, self.pan_service)
        rospy.Service('~tilt', Tilt, self.tilt_service)
        rospy.Service('~pan_tilt', PanTilt, self.pan_tilt)

        PWM.start(self.tilt_pin, self.tilt_neutral_duty_cycle, self.frequency)
        PWM.start(self.pan_pin, self.pan_neutral_duty_cycle, self.frequency)
        
    def reset(self, req):
        self.pan(0)
        self.tilt(0)
        
        return ResetResponse()

    def pan_tilt(self, req):
        self.pan(req.panAngle)
        self.tilt(req.tiltAngle)
        
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
        
        return TiltResponse()
        
    def stop(self):
        PWM.stop(self.pan_pin) 
        PWM.stop(self.tilt_pin)
        PWM.cleanup()
                            
if __name__ == '__main__':
    rate = rospy.Rate(10.0)
    ptu = PTU()

    while not rospy.is_shutdown():
        ptu.tf_publisher.sendTransform((0, 0, ptu.tilt_arm_height),
                                        tf.transformations.quaternion_from_euler(0, ptu.tilt_value, ptu.pan_value),
                                        rospy.Time.now(),
                                        "ptu",
                                        "base_ptu")

        rate.sleep()

    ptu.stop()
    
    