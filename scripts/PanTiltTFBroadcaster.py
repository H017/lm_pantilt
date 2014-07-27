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
    pan_height = rospy.get_param('~pan_height', 0.062)
    tilt_height = rospy.get_param('~tilt_height', 0.038)

    rospy.Subscriber('/ptu/state',
                     PanTiltState,
                     pantilt_state_callback)

    broadcaster = tf.TransformBroadcaster()
    r = rospy.Rate(50)

    while not rospy.is_shutdown():

        broadcaster.sendTransform((0, 0, pan_height),
            tf.transformations.quaternion_from_euler(0, -tilt_rotation, pan_rotation),
            rospy.Time.now(),
            "ptu_pan_tilt",
            "ptu")

        broadcaster.sendTransform((0, 0, tilt_height),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "ptu_attachment",
            "ptu_pan_tilt")

        r.sleep()
