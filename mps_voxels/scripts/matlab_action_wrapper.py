#! /usr/bin/env python

from __future__ import print_function
import time
import rospy
import actionlib
import importlib
# from mps_msgs.msg import ClusterRigidMotionsAction
from actionlib_msgs.msg import GoalStatus


class ActionForwarder(object):
    def __init__(self, name, pkg_name, action_name, request_name, response_name):
        try:
            self.pkg = importlib.import_module('.msg', package=pkg_name)
        except ImportError as err:
            print('Package Import Error:', err)
            exit(-1)
        print(self.pkg)
        self.action = getattr(self.pkg, action_name + 'Action')
        self.actionGoal = getattr(self.pkg, action_name + 'Goal')
        self.actionResult = getattr(self.pkg, action_name + 'Result')
        print(self.action)

        self.pub = rospy.Publisher(request_name, self.actionGoal, queue_size=1)
        self.sub = rospy.Subscriber(response_name, self.actionResult, callback=self.response_cb, queue_size=1)

        self.got_response = False
        self.response = None

        self.action_name = name
        self.server = actionlib.SimpleActionServer(self.action_name, self.action,
                                                   execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

    def execute_cb(self, goal):
        print('Sending Goal')
        if self.sub.get_num_connections() == 0:
            print('Response topic not active. Aborting.')
            self.server.set_aborted(result=None, text='Response topic not active. Aborting.')
            return

        self.got_response = False
        self.pub.publish(goal)
        start_time = rospy.Time.now()
        polling = rospy.Rate(10, reset=True)
        while not self.got_response and not rospy.is_shutdown():
            polling.sleep()
            if rospy.Time.now() - start_time > rospy.Duration(20):
                print('Timeout Failure.')
                status = self.server.current_goal.get_goal_status()
                if status == GoalStatus.ACTIVE or status == GoalStatus.PENDING:
                    self.server.set_aborted(result=None, text='Response timed out.')
                    break
        if self.got_response:
            print('Success!')
            self.server.set_succeeded(self.response)

    def response_cb(self, response):
        self.response = response
        self.got_response = True


if __name__ == '__main__':
    rospy.init_node('action_forwarder')
    pkgName = rospy.get_param('~pkg')  # e.g. 'mps_msgs'
    actionName = rospy.get_param('~action')  # e.g. 'ClusterRigidMotions'
    requestName = rospy.get_param('~request')
    responseName = rospy.get_param('~response')
    server = ActionForwarder(rospy.get_name(), pkgName, actionName, requestName, responseName)

    # Reset the action server if time is reset
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        try:
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            server = ActionForwarder(rospy.get_name(), pkgName, actionName, requestName, responseName)
