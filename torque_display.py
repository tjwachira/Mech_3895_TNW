#!/usr/bin/env python3

import rospy
from franka_msgs.msg import FrankaState

def franka_state_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " tau_ext_hat_filtered values: %s", data.tau_ext_hat_filtered)

def franka_state_subscriber():
    rospy.init_node('franka_state_subscriber', anonymous=True)

    rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, franka_state_callback)

    #rospy.spin()

if __name__ == '__main__':
    try:
        franka_state_subscriber()
    except rospy.ROSInterruptException:
        pass
