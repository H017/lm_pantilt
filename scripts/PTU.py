#!/usr/bin/env python

from lm_pantilt.srv import Start, StartResponse, Stop, StopResponse, Reset, ResetResponse, Pan, PanResponse, Tilt, TiltResponse, PanTilt, PanTiltResponse
from lm_pantilt.msg import PanTiltState
from roscpp.srv import GetLoggers

import rospy
import Adafruit_BBIO.PWM as PWM

class PTU:
    def __init__(self):
        rospy.init_node('ptu')

        self.started = False

        # Pan parameters
        self.pan_pin = rospy.get_param('~pan_pin', 'P8_13')
        self.pan_neutral_duty_cycle = rospy.get_param('~pan_neutral_duty_cycle', 7.4)
        self.pan_min_duty_cycle = rospy.get_param('~pan_min_duty_cycle', 2.7)
        self.pan_max_duty_cycle = rospy.get_param('~pan_max_duty_cycle', 12.1)
        self.pan_min_angle = rospy.get_param('~pan_min_angle', -90)
        self.pan_max_angle = rospy.get_param('~pan_max_angle', 90)

        # Tilt parameters
        self.tilt_pin = rospy.get_param('~tilt_pin', 'P9_14')
        self.tilt_neutral_duty_cycle = rospy.get_param('~tilt_neutral_duty_cycle', 11.03)
        self.tilt_min_duty_cycle = rospy.get_param('~tilt_min_duty_cycle', 6.2)
        self.tilt_max_duty_cycle = rospy.get_param('~tilt_max_duty_cycle', 12.66)
        self.tilt_min_angle = rospy.get_param('~tilt_min_angle', -30)
        self.tilt_max_angle = rospy.get_param('~tilt_max_angle', 90)

        # Global PWM parameters
        self.frequency = rospy.get_param('~frequency', 50)

        # PTU angle publisher parameters
        self.ptu_publisher = rospy.Publisher('/ptu/state', PanTiltState, queue_size=10, latch=True)
        self.pantilt_state = PanTiltState()
        self.pantilt_state.pan = 0
        self.pantilt_state.tilt = 0

        # Services
        rospy.Service('~start', Start, self.start_service)
        rospy.Service('~stop', Stop, self.stop_service)
        rospy.Service('~reset', Reset, self.reset)
        rospy.Service('~pan', Pan, self.pan_service)
        rospy.Service('~tilt', Tilt, self.tilt_service)
        rospy.Service('~pan_tilt', PanTilt, self.pan_tilt)
        
    def reset(self, req):
        response = ResetResponse()
        response.success = True

        if self.started:
            self.pan(0)
            self.tilt(0)

            self.publish_ptu_state(0, 0)
        else:
            rospy.logerr("PTU not started. Call the 'start' service.")
            response.success = False
        
        return response

    def pan_tilt(self, req):
        response = PanTiltResponse()
        response.success = True

        if self.started:
            self.pan(req.panAngle)
            self.tilt(req.tiltAngle)

            self.publish_ptu_state(req.panAngle, req.tiltAngle)
        else:
            rospy.logerr("PTU not started. Call the 'start' service.")
            response.success = False
        
        return response

    def pan(self, angle):
        if(angle > self.pan_max_angle):
            angle = self.pan_max_angle
            rospy.loginfo("Angle requested is greater than the max allowed angle.")
        elif(angle < self.pan_min_angle):
            angle = self.pan_min_angle
            rospy.loginfo("Angle requested is less than the min allowed angle.")
            
        PWM.set_duty_cycle(self.pan_pin, -angle * ((self.pan_max_duty_cycle - self.pan_min_duty_cycle) / (self.pan_max_angle - self.pan_min_angle)) + self.pan_neutral_duty_cycle)
        self.pan_angle = angle

    def pan_service(self, req):
        response = PanResponse()
        response.success = True

        if self.started:
            self.pan(req.angle)
            self.publish_ptu_state(pan=req.angle)
        else:
            rospy.logerr("PTU not started. Call the 'start' service.")
            response.success = False
        
        return response

    def tilt(self, angle):
        if(angle > self.tilt_max_angle):
            angle = self.tilt_max_angle
            rospy.loginfo("Angle requested is greater than the max allowed angle.")
        elif(angle < self.tilt_min_angle):
            angle = self.tilt_min_angle
            rospy.loginfo("Angle requested is less than the min allowed angle.")

        PWM.set_duty_cycle(self.tilt_pin, -angle * ((self.tilt_max_duty_cycle - self.tilt_min_duty_cycle) / (self.tilt_max_angle - self.tilt_min_angle)) + self.tilt_neutral_duty_cycle)
        self.tilt_angle = angle

    def tilt_service(self, req):
        response = TiltResponse()
        response.success = True

        if self.started:
            self.tilt(req.angle)
            self.publish_ptu_state(tilt=req.angle)
        else:
            rospy.logerr("PTU not started. Call the 'start' service.")
            response.success = False
        
        return response

    def publish_ptu_state(self, pan=None, tilt=None):
        if(pan is not None):
            self.pantilt_state.pan = pan

        if(tilt is not None):
            self.pantilt_state.tilt = tilt

        self.ptu_publisher.publish(self.pantilt_state)

    def stop_service(self, req):
        response = StopResponse()
        response.success = False

        if self.started:
            self.stop()
            response.success = True

        return response

    def stop(self):
        PWM.stop(self.pan_pin)
        PWM.stop(self.tilt_pin)
        PWM.cleanup()
        self.started = False

    def start_service(self, req):
        response = StartResponse()
        response.success = False

        if not self.started:
            self.start()
            response.success = True

        return response

    def start(self):
        PWM.start(self.tilt_pin, self.tilt_neutral_duty_cycle, self.frequency)
        PWM.start(self.pan_pin, self.pan_neutral_duty_cycle, self.frequency)
        self.publish_ptu_state(0, 0)
        self.started = True
                            
if __name__ == '__main__':
    ptu = PTU()
    ptu.start()

    # Check if the core is still up
    last_core_check = rospy.get_time()
    get_loggers = rospy.ServiceProxy("/rosout/get_loggers", GetLoggers)
    try:
        while not rospy.is_shutdown():
            if rospy.get_time() - last_core_check > 5:
                last_core_check = rospy.get_time()
                get_loggers()
            rospy.sleep(1)
    except rospy.ServiceException:
        print "Lost communication with the core. Stopping..."
            
    ptu.stop()
    
    