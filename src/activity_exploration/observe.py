#! /usr/bin/env python

import yaml
import rospy
import smach
import roslib
import actionlib
from human_trajectory.msg import Trajectories
from vision_people_logging.srv import CaptureUBD
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
# from record_skeletons_action.msg import skeletonAction, skeletonGoal

from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import AddTasks
from strands_executive_msgs import task_utils as tu

class Observe(smach.State):

    def __init__(self):
        rospy.loginfo('Initiating observe state...')
        smach.State.__init__(
            self, outcomes=['succeeded'], input_keys=['waypoint', 'roi'],
            output_keys=["waypoint", 'roi']
        )

        self.soma_config = rospy.get_param("~soma_config", "activity_exploration")
        self._is_observing = False
        self.nav_client = actionlib.SimpleActionClient(
            'topological_navigation', GotoNodeAction
        )
        self.nav_client.wait_for_server()
        rospy.loginfo("Connected to topological_navigation action server...")
        self.observe_wps = yaml.load(
            open(
                roslib.packages.get_pkg_dir('activity_exploration') + '/config/type_to_wp.yaml',
                'r'
            )
        )['observe']
        self.observe_duration = rospy.Duration(
            rospy.get_param("~observe_duration", 1200)
        )
        self._reset_minute_check(self.observe_duration)
        rospy.Subscriber(
            "/people_trajectory/trajectories/batch", Trajectories, self._pt_cb, None, 10
        )
        # add action client to paul stuff
        # self.action_client = actionlib.SimpleActionClient(
        #     '/record_skeletons', skeletonAction
        # )
        # self.action_client.wait_for_server()
        # rospy.loginfo("Connected to /record_skeletons action server...")
        # add service addTasks
        self.add_tasks_srv = rospy.ServiceProxy('/task_executor/demand_tasks', AddTasks)
        self.add_tasks_srv.wait_for_service()
        rospy.loginfo("Connected to /task_executor/demand_tasks service...")
        self.ubd_srv = rospy.ServiceProxy("/vision_logging_service/capture", CaptureUBD)
        self.ubd_srv.wait_for_service()
        rospy.loginfo("Connected to /vision_logging_service/capture service...")

    def _pt_cb(self, msg):
        if self._is_observing:
            for trajectory in msg.trajectories:
                start = trajectory.start_time
                ind = ((start - self._start_time).secs / 60) % len(self.minute_check)
                if ind not in self._indices:
                    rospy.loginfo("A person is detected at %d" % start.secs)
                    self.minute_check[ind] = True
                    self._indices.append(ind)
                    break

    def _reset_minute_check(self, duration):
        self._indices = list()
        self._start_time = rospy.Time.now()
        self.minute_check = [
            False for i in range(0, duration.secs/(60*2))
        ]

    def execute(self, userdata):
        rospy.loginfo("Entering observe mode...")
        start = rospy.Time.now()
        nav_goal = GotoNodeGoal()
        nav_goal.target = userdata.waypoint
        self.nav_client.send_goal(nav_goal)
        self.nav_client.wait_for_result()
        self._reset_minute_check(self.observe_duration)
        rospy.sleep(0.1)
        self._is_observing = True
        end = rospy.Time.now()
        duration_left = self.observe_duration - (end - start)
        if duration_left > rospy.Duration(1, 0):
            task = Task(
                action="skeleton_action",
                start_node_id=userdata.waypoint,
                end_node_id=userdata.waypoint,
                start_after=start,
                end_before=start+self.observe_duration,
                max_duration=duration_left
            )
            tu.add_duration_argument(task, duration_left)
            tu.add_string_argument(task, userdata.roi)
            tu.add_string_argument(task, self.soma_config)
            rospy.sleep(1)
            rospy.loginfo("Adding a task to task scheduler from %d to %d" % (start.secs, (start+self.observe_duration).secs))
            self.add_tasks_srv([task])
            # self.action_client.send_goal(
            #     skeletonGoal(duration_left, userdata.roi, self.soma_config)
            # )
            is_person_there = False
            while (rospy.Time.now() - start) < duration_left and not rospy.is_shutdown():
                if False not in self.minute_check and not is_person_there:
                    rospy.loginfo("Humans are constantly detected for %d minutes" % len(self.minute_check))
                    self._is_observing = False
                    is_person_there = True
                    self.ubd_srv()
                # if self.action_client.get_state() in [
                #     actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.ABORTED
                # ]:
                #     break
                rospy.sleep(1)
            # self.action_client.wait_for_result()
        # if False in self.minute_check:
        # Need to capture a pic
        userdata.waypoint = ''
        userdata.roi = ''
        self._is_observing = False
        return 'succeeded'
