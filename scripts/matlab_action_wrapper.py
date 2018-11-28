#! /usr/bin/env python

from __future__ import print_function
import time
import rospy
import actionlib
import importlib
# from mps_msgs.msg import ClusterRigidMotionsAction


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
        print('Sending Goal: ', goal)
        if self.sub.get_num_connections() == 0:
            print('Response topic not active. Aborting.')
            self.server.set_aborted()
            return

        self.got_response = False
        self.pub.publish(goal)
        while not self.got_response:
            time.sleep(0.1)
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
    rospy.spin()
