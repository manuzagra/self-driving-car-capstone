#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray
from std_msgs.msg import Int32
import math
import copy

import numpy as np
from scipy.spatial import KDTree

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number



class WaypointUpdater(object):
    def __init__(self):

        ##### member variables
        # they are first to ensure they exist before any callback is called
        self.pose = None
        self.speed = None
        self.base_waypoints_2d_tree = None
        self.next_stop_idx = -1

        ##### ROS stuff
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO done: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)  # this is Int32 for sure

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        # do not run enything till the moment we have messages
        rospy.wait_for_message('/base_waypoints', Lane)
        rospy.wait_for_message('/current_pose', PoseStamped)

        # TODO only for test, delete this subscriber
        # rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_test_cb)  # teest only

        self.loop()  # program will stay here forever

    def pose_cb(self, msg):
        self.pose = msg

    def velocity_cb(self, msg):
        self.speed = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        self.base_waypoints_2d_tree = KDTree([[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in waypoints.waypoints])  # tree taken from the video

    def traffic_cb(self, msg):
        self.next_stop_idx = msg.data

    # TODO only for test, delete this method
    def traffic_test_cb(self, msg):
        self.stops_idx = []
        for tl in msg.lights:
            if tl.state != 2:
                x = tl.pose.pose.position.x
                y = tl.pose.pose.position.y
                self.stops_idx.append(self.base_waypoints_2d_tree.query([x, y], 1)[1])

        current_idx = self.next_waypoint_index()
        stops_in_range = [stop_idx for stop_idx in self.stops_idx if current_idx < stop_idx < current_idx+LOOKAHEAD_WPS]

        if len(stops_in_range):
            self.next_stop_idx = stops_in_range[0]
        else:
            self.next_stop_idx = -1

    def loop(self):
        """
        The program will stay here forever
        """

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            next_wp_idx = self.next_waypoint_index()
            lane = self.get_lane(next_wp_idx, next_wp_idx+LOOKAHEAD_WPS)
            if next_wp_idx < self.next_stop_idx < next_wp_idx+LOOKAHEAD_WPS:
                lane = self.decelerate_waypoints(lane, self.next_stop_idx-next_wp_idx)
            self.final_waypoints_pub.publish(lane)
            r.sleep()

    def next_waypoint_index(self):
        """
        It returns the next waypoint from the base_waypoints.
        """
        # get the index of the closest waypoint
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.base_waypoints_2d_tree.query([x, y], 1)[1] # [0] is the position

        # get the coordinates of the closest, the previous of the closest wp and the pose
        closest_coord = np.array([self.base_waypoints.waypoints[closest_idx].pose.pose.position.x, self.base_waypoints.waypoints[closest_idx].pose.pose.position.y])
        prev_coord = np.array([self.base_waypoints.waypoints[closest_idx-1].pose.pose.position.x, self.base_waypoints.waypoints[closest_idx-1].pose.pose.position.y])
        pose_coord = np.array([self.pose.pose.position.x, self.pose.pose.position.y])

        # I dont really understand the maths explained in the video,
        # it is a bit complicated way to do a simple thing, but they use it so...
        # the best way to understand it is to thing in terms of angles,
        # if angle between vectors > 90 -> dot product < 0
        # if angle between vectors < 90 -> dot product > 0
        val = np.dot(closest_coord-prev_coord, pose_coord-closest_coord)

        next_waypoint_idx = closest_idx
        if val > 0:
            next_waypoint_idx = (next_waypoint_idx + 1) % len(self.base_waypoints.waypoints)
        return next_waypoint_idx

    def get_lane(self, from_idx, to_idx, stop_idx=None):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = [Waypoint()] * (to_idx-from_idx)
        lane.waypoints = self.base_waypoints.waypoints[from_idx:to_idx]
        return lane

    def decelerate_waypoints(self, lane, stop_idx):
        l = copy.deepcopy(lane)
        max_decel = 5
        for idx, wp in enumerate(l.waypoints):
            if idx < stop_idx:
                dist = max(0, self.distance(l.waypoints, idx, stop_idx) - 20)
                vel = math.sqrt(2*dist*max_decel)
                wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            else:
                wp.twist.twist.linear.x = 0.
        return l

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
