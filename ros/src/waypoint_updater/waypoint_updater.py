#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

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
        self.base_waypoints = waypoints
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

        # get the coordinates of the closest, the previous of the closest wp and the pose
        closest_coord = np.array([self.base_waypoints.waypoints.[closest_idx].pose.pose.position.x, self.base_waypoints.waypoints.[closest_idx].pose.pose.position.y])
        prev_coord = np.array([self.base_waypoints.waypoints.[closest_idx-1].pose.pose.position.x, self.base_waypoints.waypoints.[closest_idx-1].pose.pose.position.y])
        pose_coord = np.array([self.pose.position.x, self.pose.position.y])

        # I dont really understand the maths explained in the video,
        # it is a bit complicated way to do a simple thing, but they use it so...
        # the best way to understand it is to thing in terms of angles,
        # if angle between vectors > 90º -> dot product < 0
        # if angle between vectors < 90º -> dot product > 0
        val = np.dot(closest_coord-prev_coord, pose_coord-closest_coord)

        next_waypoint_idx = closest_idx
        if val > 0:
            next_waypoint_idx = (next_waypoint_idx + 1) % len(self.base_waypoints.waypoints)
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
