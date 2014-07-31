#!/usr/bin/env python
from lm_pantilt.msg import PanTiltState
import rospy
import tf
import math
import numpy

def pantilt_state_callback(pantilt_state):
    global pan_rotation, tilt_rotation

    tilt_rotation = pantilt_state.tilt * (math.pi / 180)
    pan_rotation = pantilt_state.pan * (math.pi / 180)
   
 
if __name__ == '__main__':
    rospy.init_node('pan_tilt_tf_broadcaster')

    pan_rotation = 0
    tilt_rotation = 0
    pan_height = rospy.get_param('~pan_height', 0.062)
    tilt_height = rospy.get_param('~tilt_height', 0.038)

    rospy.Subscriber('/ptu/state',
                     PanTiltState,
                     pantilt_state_callback)

    tfb = tf.TransformBroadcaster()
    r = rospy.Rate(20)    

    while not rospy.is_shutdown():
        t = tf.TransformerROS()
        t1 = t.fromTranslationRotation((0, 0, pan_height), tf.transformations.quaternion_from_euler(0, -tilt_rotation, pan_rotation))
        t2 = t.fromTranslationRotation((0, 0, tilt_height),tf.transformations.quaternion_from_euler(0, 0, 0))    
        t3 = numpy.dot(t1, t2)
        translation = tf.transformations.translation_from_matrix(t3)
        rotation = tf.transformations.quaternion_from_matrix(t3)
        tfb.sendTransform(translation, rotation, rospy.Time.now(), "ptu_attachment", "ptu")
        r.sleep()                                                    
