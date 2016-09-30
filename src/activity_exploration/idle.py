#! /usr/bin/env python

import yaml
import rospy
import smach
import roslib
import random
from human_trajectory.msg import Trajectories
from strands_exploration_msgs.srv import GetExplorationTasks
from region_observation.util import create_line_string, is_intersected, get_soma_info


class Idle(smach.State):

    def __init__(self):
        rospy.loginfo('Initiating idle state...')
        smach.State.__init__(
            self, outcomes=['patrol', 'observe', 'aborted'],
            input_keys=['waypoint'], output_keys=["waypoint"]
        )
        self.probability = rospy.get_param("~patrol_probability", 0.7)
        self.idle_duration = rospy.Duration(
            rospy.get_param("~idle_duration", 300)
        )
        self.observe_duration = rospy.Duration(
            rospy.get_param("~observe_duration", 1200)
        )
        self._is_counting_visit = False
        self._region_visits = dict()
        self.regions, _ = get_soma_info(
            rospy.get_param("~soma_config", "activity_exploration")
        )
        self.region_wps = yaml.load(
            open(
                roslib.packages.get_pkg_dir('activity_exploration') + '/config/region_to_wp.yaml',
                'r'
            )
        )
        rospy.loginfo("Region to WayPoint relation: %s" % str(self.region_wps))
        self.type_wps = yaml.load(
            open(
                roslib.packages.get_pkg_dir('activity_exploration') + '/config/type_to_wp.yaml',
                'r'
            )
        )
        rospy.loginfo("Type of actions to WayPoints: %s" % str(self.type_wps))
        
        if self.probability < 1.0:
            rospy.loginfo("Connecting to /arms/activity_rcmd_srv...")
            self.recommender_srv = rospy.ServiceProxy(
                "/arms/activity_rcmd_srv", GetExplorationTasks
            )
            self.recommender_srv.wait_for_service()
        rospy.loginfo("Subscribing to /people_trajectory/trajectories/batch...")
        rospy.Subscriber(
            "/people_trajectory/trajectories/batch", Trajectories, self._pt_cb, None, 10
        )

    def _pt_cb(self, msg):
        if self._is_counting_visit:
            for trajectory in msg.trajectories:
                points = [
                    [
                        pose.pose.position.x, pose.pose.position.y
                    ] for pose in trajectory.trajectory[-10:]
                ]
                points = create_line_string(points)
                for roi, area in self.regions.iteritems():
                    if is_intersected(area, points):
                        if trajectory.uuid not in self._region_visits:
                            rospy.loginfo("%s seems to visit %s" % (trajectory.uuid, roi))
                            self._region_visits[trajectory.uuid] = roi
                        break

    def execute(self, userdata):
        random.seed(rospy.Time.now())
        next_state = 'observe'
        rospy.loginfo("Entering idle mode...")
        self._region_visits = dict()
        self._is_counting_visit = True
        rospy.sleep(self.idle_duration)
        if random.random() <= self.probability:
            next_state = 'patrol'
            userdata.waypoint = self.type_wps[next_state][
                random.randint(0, len(self.type_wps[next_state])-1)
            ]
        else:
            userdata.waypoint = self.get_recommended_places()

        if userdata.waypoint == "":
            rospy.logwarn("No waypoint is set!!!")
            next_state = "aborted"
        else:
            rospy.loginfo(
                "Leaving idle mode, next state is %s visiting %s..." % (
                    next_state, userdata.waypoint
                )
            )
        self._is_counting_visit = False
        return next_state

    def get_recommended_places(self, next_state='observe'):
        # get recommendation from activity recommender system
        start = rospy.Time.now()
        end = start + self.observe_duration
        places = self.recommender_srv(start, end)
        if len(places.task_definition) == 0:
            return self.type_wps[next_state][
                random.randint(0, len(self.type_wps[next_state])-1)
            ]
        ind = random.randint(
            0, min([2, len(places.task_definition)-1])
        )
        waypoint = places.task_definition[ind]
        wp_score = places.task_score[ind]
        # with some prob, compare the recommended place with current observed
        # people, which one higher is chosen.
        if random.random() <= self.probability:
            rois = dict()
            for _, roi in self._region_visits.iteritems():
                if roi not in rois:
                    rois[roi] = 0
                rois[roi] += 1
            temp = sorted(rois, key=lambda i: rois[i], reverse=True)
            for roi in temp:
                if roi in self.type_wps[next_state] and rois[roi] > wp_score:
                    waypoint = self.region_wps[roi]
        return waypoint
