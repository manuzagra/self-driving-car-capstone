#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):

        ##### member variables
        # they are first to ensure they exist before any callback is called
        self.pose = None
        self.base_waypoints_2d_tree = None

        ##### ROS stuff
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO done: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)  # this is Int32 for sure
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)  # no idea of this type

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        self.run()  # program will stay here forever

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints   # get rid of the header
        self.base_waypoints_2d_tree = KDTree([[wp.pose.pose.position.x wp.pose.pose.position.y] for wp in waypoints.waypoints])  # tree taken from the video

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def run(self):
        """
        The program will stay here forever
        """
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            pass # here we do the processing
            r.sleep()

    def next_waypoint_index(self):
        """
        It returns the next waypoint from the base_waypoints.
        """
        # get the index of the closest waypoint
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.base_waypoints_2d_tree.query([x, y], 1)[1] # [0] is the position

        # check if the closest wp is ahead or behind the car
        closest_wp = self.base_waypoints[closest_idx]
        before_closest_wp = self.base_waypoints[closest_idx-1]

        # TODO this supposse the car is alway travelling in ascending ondex of wps
        # I am going to check with distances
        distance = lambda p1, p2: math.sqrt((p1.position.x-p2.position.x)**2 + (p1.position.y-p2.position.y)**2)
        closest_wp_dist = distance(closest_wp, before_closest_wp)
        pose_dist = distance(self.pose, before_closest_wp)

        ###    before_closest_wp   car   closest_wp
        next_waypoint_idx = closest_idx
        if (pose_dist > closest_wp_dist):
            ###    before_closest_wp   closest_wp   car   after_closest_wp
            next_waypoint_idx += 1
        return next_waypoint_idx


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
