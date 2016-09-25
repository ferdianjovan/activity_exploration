#! /usr/bin/env python

import yaml
import rospy
import smach
import roslib
import random
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal


class Patrol(smach.State):

    def __init__(self):
        rospy.loginfo('Initiating patrol state...')
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['waypoint'])

        rospy.loginfo("Connecting to topological_navigation action server...")
        self.nav_client = actionlib.SimpleActionClient(
            'topological_navigation', GotoNodeAction
        )
        self.nav_client.wait_for_server()
        self.patrol_wps = yaml.load(
            open(
                roslib.packages.get_pkg_dir('activity_exploration') + '/config/type_to_wp.yaml',
                'r'
            )
        )['patrol']
        self.patrol_duration = rospy.Duration(
            rospy.get_param("~observe_duration", 1200)
        )

    def execute(self, userdata):
        rospy.loginfo("Entering patrol mode...")
        start = rospy.Time.now()
        visited_wps = list()
        current_wp = userdata.waypoint

        while (rospy.Time.now() - start) < self.patrol_duration and not rospy.is_shutdown():
            self.go_to(current_wp)
            visited_wps.append(current_wp)
            unvisited = [
                wp for wp in self.patrol_wps if wp not in visited_wps
            ]
            if len(unvisited) == 0:
                current_wp = self.patrol_wps[
                    random.randint(0, len(self.patrol_wps)-1)
                ]
            else:
                current_wp = unvisited[random.randint(0, len(unvisited)-1)]
            rospy.sleep(int(self.patrol_duration.secs * 0.1))
        return 'succeeded'

    def go_to(self, waypoint):
        nav_goal = GotoNodeGoal()
        nav_goal.target = waypoint
        self.nav_client.send_goal(nav_goal)
        self.nav_client.wait_for_result(
            timeout=rospy.Duration(int(self.patrol_duration.secs * 0.1))
        )
