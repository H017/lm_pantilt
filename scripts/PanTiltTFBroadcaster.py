#!/usr/bin/env python
from lm_pantilt.msg import PanTiltState
import rospy
import tf
import math

def pantilt_state_callback(pantilt_state):
    global pan_rotation, tilt_rotation

    tilt_rotation = pantilt_state.tilt * (math.pi / 180)
    pan_rotation = pantilt_state.pan * (math.pi / 180)

if __name__ == '__main__':
    rospy.init_node('pan_tilt_tf_broadcaster')

    pan_rotation = 0
    tilt_rotation = 0
    pan_translation = rospy.get_param('~pan_translation', 1.5) # Need to test
    tilt_translation = rospy.get_param('~tilt_translation', 0.042)

    rospy.Subscriber('/ptu/state',
                     PanTiltState,
                     pantilt_state_callback)

    pan_frame_broadcaster = tf.TransformBroadcaster()
    tilt_frame_broadcaster = tf.TransformBroadcaster()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pan_frame_broadcaster.sendTransform((0, 0, pan_translation),  # Need to test
            tf.transformations.quaternion_from_euler(0, 0, pan_rotation), # Need to test
            rospy.Time.now(),
            "pan_frame",
            "base_link")

        tilt_frame_broadcaster.sendTransform((0, 0, tilt_translation),
            tf.transformations.quaternion_from_euler(0, tilt_rotation, 0), # Need to test
            rospy.Time.now(),
            "tilt_frame",
            "pan_frame")

        r.sleep()
