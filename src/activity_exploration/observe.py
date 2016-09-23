#! /usr/bin/env python

import yaml
import rospy
import smach
import roslib
import actionlib
from human_trajectory.msg import Trajectories
from vision_people_logging.srv import CaptureUBD
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal


class Observe(smach.State):

    def __init__(self):
        rospy.loginfo('Initiating observe state...')
        smach.State.__init__(
            self, outcomes=['succeeded', 'aborted'],
            input_keys=['waypoint']
        )

        self._reset_minute_check()  # for checking whether person is there for n consecutive minutes
        rospy.loginfo("Connecting to topological_navigation action server...")
        self.nav_client = actionlib.SimpleActionClient(
            'topological_navigation', GotoNodeAction
        )
        self.nav_client.wait_for_server()
        self.observe_wps = yaml.load(
            open(
                roslib.packages.get_pkg_dir('activity_exploration') + '/config/type_to_wp.yaml',
                'r'
            )
        )['observe']
        self.observe_duration = rospy.Duration(
            rospy.get_param("~observe_duration", 1200)
        )
        rospy.Subscriber(
            "/people_trajectory/trajectories/batch", Trajectories, self._pt_cb, None, 10
        )
        # rospy.loginfo("Connecting to topological_navigation action server...")
        # add action client to paul stuff
        # self.action_client = actionlib.SimpleActionClient(
        #     '', SomeAction
        # )
        # self.action_client.wait_for_server()
        self.ubd_srv = rospy.ServiceProxy("/vision_logging_service/capture", CaptureUBD)

    def _pt_cb(self, msg):
        if self._is_observing:
            for trajectory in msg.trajectories:
                start = trajectory.start_time
                if (start - self._start_time).secs / 60 == self._counter:
                    self.minute_check[self.counter % len(self.minute_check)] = True
                    self._counter += 1
                    break

    def _reset_minute_check(self):
        self._counter = 0
        self._start_time = rospy.Time.now()
        self.minute_check = [
            False for i in rospy.get_param("~observe_duration", 1200)/(60*2)
        ]

    def execute(self, userdata):
        rospy.loginfo("Entering observe mode...")
        start = rospy.Time.now()
        nav_goal = GotoNodeGoal()
        nav_goal.target = userdata.waypoint
        self.nav_client.send_goal(nav_goal)
        self.nav_client.wait_for_result()
        self._reset_minute_check()
        rospy.sleep(0.1)
        self._is_observing = True

        is_person_there = False
        while (rospy.Time.now() - start) < self.observe_duration and not rospy.is_shutdown():
            if False not in self.minute_check and not is_person_there:
                rospy.loginfo("Humans are constantly detected for %d minutes" % len(self.minute_check))
                self._is_observing = False
                is_person_there = True
            rospy.sleep(1)

        if False in self.minute_check:
            self.ubd_srv()
        self._is_observing = False
        return 'succeeded'
