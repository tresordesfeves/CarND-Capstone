#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math
import numpy as np

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
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        #initializing all variable to enable first conditional excution
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None


        self.loop()

    def loop(self):
        rate =rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # find the waypoints closest to the car's current positiom
                closest_waypoints_idx=self.get_closest_waypoints_idx()
                self.publish_waypoints(closest_waypoints_idx)
            rate.sleep()

    def get_closest_waypoints_idx(self):
        x=self.pose.pose.position.x
        y=self.pose.pose.position.y

        # querry the KDTree with the vehicle current position to find the closest waypoint index
        closest_idx=self.waypoint_tree.query([x,y],1)[1] # ,1 : the closest one, [1], the index in waypoints_2d list

        closest_coord=self.waypoints_2d[closest_idx] # coordinates of the closest waypoint 
        prev_coord=self.waypoints_2d[closest_idx-1] # coordinates of the previous point in the waypoints sequence       

        close_waypoints_vector = np.array(closest_coord) - np.array(prev_coord) # close waypoints vector 
        closest_to_position_vector=np.array([x,y]) - np.array(closest_coord) # vector closest waypoint to vehicle current position      

        check_dotprod = np.dot(close_waypoints_vector,closest_to_position_vector)       

        if check_dotprod > 0:
            # vehicle is ahead of the closest waypoint: chose the next waypoint to start the sequence of waypoints ahead of the vehicle
            closest_idx= (closest_idx+1) % (len(self.waypoints_2d))      

        return closest_idx

    def publish_waypoints(self,closest_idx):
        lane=Lane()
        lane.header=self.base_waypoints.header # reusing the header from msa type  "Lane" also used for base_waypoint 
        lane.waypoints=self.base_waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose=msg
        pass

    def waypoints_cb(self, waypoints):

        self.base_waypoints=waypoints
        if not self.waypoints_2d:
            self.waypoints_2d=[[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree= KDTree(self.waypoints_2d)
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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




