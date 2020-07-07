#! /usr/bin/env python

"""
ros_trajectory_follower.py

Executes a ROS JointTrajectory as a message or action on Victor.

Author: Andrew Price
Date: 2018-06-06
"""

import sys
import rospy
import actionlib
from std_srvs.srv import SetBool, SetBoolResponse
from trajectory_msgs.msg import JointTrajectory
import control_msgs.msg


class TrajectoryForwarder(object):
    def __init__(self):
        self._action_name = "follow_joint_trajectory"

        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        res = control_msgs.msg.FollowJointTrajectoryResult()
        raw_input("Press Enter to Continue...")
        res.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
        self._as.set_succeeded(res)


def main(argv):
    rospy.init_node('fake_trajectory_forwarder', argv=argv)

    fwd = TrajectoryForwarder()

    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)
