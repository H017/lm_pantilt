#!/usr/bin/env python
from lm_pantilt.msg import PanTiltState
import rospy
import tf

def broadcast_tf(pantilt_state):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, offset),                                                                     # Need to test
                     tf.transformations.quaternion_from_euler(0, pantilt_state.tilt, pantilt_state.pan), # Need to test
                     rospy.Time.now(),
                     "ptu_frame",
                     "base_link")

if __name__ == '__main__':
    rospy.init_node('pan_tilt_tf_broadcaster')
    offset = rospy.get_param('~offset', 1.5) # Need to test
    rospy.Subscriber('/ptu/state',
                     PanTiltState,
                     broadcast_tf)
    rospy.spin()