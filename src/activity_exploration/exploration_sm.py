#! /usr/bin/env python

import rospy
import smach
from activity_exploration.idle import Idle
from activity_exploration.patrol import Patrol
from activity_exploration.observe import Observe


class ExplorationSM(smach.StateMachine):

    def __init__(self):
        rospy.loginfo("Starting activity exploration state machine...")
        smach.StateMachine.__init__(
            self,
            outcomes=['succeeded', 'aborted']
        )
        self.userdata.roi = ""
        self.userdata.waypoint = ""
        self._idle = Idle()
        self._patrol = Patrol()
        self._observe = Observe()

        with self:
            smach.StateMachine.add(
                'Idle', self._idle,
                transitions={
                    'patrol': 'Patrol',
                    'observe': 'Observe',
                    'aborted': 'Idle'
                }
            )
            smach.StateMachine.add(
                'Patrol', self._patrol,
                transitions={'succeeded': 'Idle'}
            )
            smach.StateMachine.add(
                'Observe', self._observe,
                transitions={'succeeded': 'Idle'}
            )
