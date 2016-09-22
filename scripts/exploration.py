#! /usr/bin/env python

import rospy
from activity_exploration.exploration_sm import ExplorationSM


class ActivityExploration(object):

    def __init__(self):
        rospy.loginfo('Initiating Activity Exploration...')
        self.sm = ExplorationSM()
        self.sm.set_initial_state(['Idle'])

    def explore(self):
        self.sm.execute()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('activity_exploration')
    os = ActivityExploration()
    os.explore()
