#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import numpy as np
import math

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number

#MAX_DECEL = rospy.get_param('~decel_limit', -5)
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        #self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        #initializing all variable to enable first conditional excution
        self.pose = None
        self.base_lane = None
        self.waypoints_2d = None
        self.stopline_wp_idx = -1 # need to be initialized (tested in "generate_lane" before first traffic_cb call)

        self.loop()


    def loop(self):
        rate =rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                # find the waypoints closest to the car's current positiom
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
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

    def publish_waypoints(self):

        speed_active_lane=self.generate_lane() # a range of speed reactive (to red traffic lights) waypoints, ahead of the car

        self.final_waypoints_pub.publish(speed_active_lane) 

    def generate_lane(self): # edit the speed of a range of waypoints ahead of the car to stop at a qualifying red light stop line 
        lane=Lane()

        closest_waypoint_idx=self.get_closest_waypoint_idx()# closest waypoint ahead of the vehicle
        farthest_waypoint_idx=closest_waypoint_idx + LOOKAHEAD_WPS # last waypoint of the lookup range ahead of the vehicle 
        
        base_waypoints= self.base_lane.waypoints[closest_waypoint_idx:farthest_waypoint_idx] #  range of waypoint in front of the car 

        # the position of each points in base_waypoints is not to be modified !!!
        # only the velocity attached to each point needs to be modify to control the car speed when aprroaching a red-light stop-line

        if (self.stopline_wp_idx ==-1) or (self.stopline_wp_idx>=farthest_waypoint_idx):# unknow traffic line status or light is too far out
            lane.waypoints= base_waypoints  # keep the original speed associated with each waypoints
            #rospy.logwarn("------------------>keep going  " + str(self.stopline_wp_idx) )
            if (self.stopline_wp_idx ==-1):
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import numpy as np
import math

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number

#MAX_DECEL = rospy.get_param('~decel_limit', -5)
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        #self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        #initializing all variable to enable first conditional excution
        self.pose = None
        self.base_lane = None
        self.waypoints_2d = None
        self.stopline_wp_idx = -1 # need to be initialized (tested in "generate_lane" before first traffic_cb call)

        self.loop()


    def loop(self):
        rate =rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                # find the waypoints closest to the car's current positiom
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
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

    def publish_waypoints(self):

        speed_active_lane=self.generate_lane() # a range of speed reactive (to red traffic lights) waypoints, ahead of the car

        self.final_waypoints_pub.publish(speed_active_lane) 

    def generate_lane(self): # edit the speed of a range of waypoints ahead of the car to stop at a qualifying red light stop line 
        lane=Lane()

        closest_waypoint_idx=self.get_closest_waypoint_idx()# closest waypoint ahead of the vehicle
        farthest_waypoint_idx=closest_waypoint_idx + LOOKAHEAD_WPS # last waypoint of the lookup range ahead of the vehicle 
        
        base_waypoints= self.base_lane.waypoints[closest_waypoint_idx:farthest_waypoint_idx] #  range of waypoint in front of the car 

        # the position of each points in base_waypoints is not to be modified !!!
        # only the velocity attached to each point needs to be modify to control the car speed when aprroaching a red-light stop-line

        if (self.stopline_wp_idx ==-1) or (self.stopline_wp_idx>=farthest_waypoint_idx):# unknow traffic line status or light is too far out
            lane.waypoints= base_waypoints  # keep the original speed associated with each waypoints
            #rospy.logwarn("------------------>keep going  " + str(self.stopline_wp_idx) )
            if (self.stopline_wp_idx ==-1):
                rospy.logwarn("no change : GREEN OR YELLOW : " )
            else:    
                rospy.logwarn("no change,  stopline > farthest: " + str(self.stopline_wp_idx) + " > " +str(farthest_waypoint_idx))


        else :
            #lane.waypoints= base_waypoints 
            lane.waypoints = self.decelerate_waypoints(base_waypoints,closest_waypoint_idx)  # base_waypoints location unaltered,
            #rospy.logwarn("-------------DECELERATE   " + str(self.stopline_wp_idx) )
            rospy.logwarn("-------------DECELERATE   " + str(lane.waypoints) )
            rospy.logwarn("-------------DECELERATE   " + str(base_waypoints) )

                                                                                    # but each base_waypoint velocity is slowed down toward a complete stop at the red-light stop-line 
        lane.header=self.base_lane.header # recycling the header 

        return lane


    def decelerate_waypoints(self, waypoints, closest_waypoint_idx): # calculate the speed of each waypoint from the car to the stop-line

        rospy.logwarn("-------------DECELERATE from  " + str(closest_waypoint_idx)+" to "+ str(self.stopline_wp_idx) )

        WP_car_to_stopline= max(self.stopline_wp_idx - (closest_waypoint_idx -2), 0)   # number of waypoints from the car to the stop-line (-2 for car's 1/2 length) 

        waypoints_controlled = waypoints

        for i, wp in enumerate(waypoints_controlled):
            distance_wp_to_stopline=self.distance(waypoints, i, WP_car_to_stopline) # culmulative distance between each waypoints from waypoints[i] to waypoint[WP_car_to_stopline]
                                                                                    # or 0 when i >WP_car_to_stopline  (meaning all the points beyound stop-line )
            velocity= math.sqrt(2*MAX_DECEL*distance_wp_to_stopline)

            if velocity < 1 :
                velocity=0 # stop 
            
            wp.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)

        return waypoints_controlled





    def pose_cb(self, msg):
        self.pose=msg
        

    def waypoints_cb(self, waypoints):

        self.base_lane=waypoints # all the existing waypoints
        if not self.waypoints_2d:
            self.waypoints_2d=[[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree= KDTree(self.waypoints_2d)
      

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message. 
        self.stopline_wp_idx = msg.data
        #rospy.logwarn("------------------>stopline_wp_idx: " + str(self.stopline_wp_idx) )


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):# wp1,wp2 are waypoints indexes, not waypoints per se

                                            # return the culmulative distance between each waypoints from wp1 to wp2
                                            # !!! return 0 when wp1>wp2
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            # sum of all the segments length from between 2 waypoints 
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i 
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')





            else:    
                rospy.logwarn("no change,  stopline > farthest: " + str(self.stopline_wp_idx) + " > " +str(farthest_waypoint_idx))


        else :
            #lane.waypoints= base_waypoints 
            lane.waypoints = self.decelerate_waypoints(base_waypoints,closest_waypoint_idx)  # base_waypoints location unaltered,
            #rospy.logwarn("-------------DECELERATE   " + str(self.stopline_wp_idx) )
                                                                                    # but each base_waypoint velocity is slowed down toward a complete stop at the red-light stop-line 
        lane.header=self.base_lane.header # recycling the header 

        return lane


    def decelerate_waypoints(self, waypoints, closest_waypoint_idx): # calculate the speed of each waypoint from the car to the stop-line

        rospy.logwarn("-------------DECELERATE from  " + str(closest_waypoint_idx)+" to "+ str(self.stopline_wp_idx) )

        WP_car_to_stopline= max(self.stopline_wp_idx - (closest_waypoint_idx -2), 0)   # number of waypoints from the car to the stop-line (-2 for car's 1/2 length) 

        waypoints_controlled = waypoints

        for i, wp in enumerate(waypoints_controlled):
            distance_wp_to_stopline=self.distance(waypoints, i, WP_car_to_stopline) # culmulative distance between each waypoints from waypoints[i] to waypoint[WP_car_to_stopline]
                                                                                    # or 0 when i >WP_car_to_stopline  (meaning all the points beyound stop-line )
            velocity= math.sqrt(2*MAX_DECEL*distance_wp_to_stopline)

            if velocity < 1 :
                velocity=0 # stop 
            
            wp.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)



        return waypoints_controlled





    def pose_cb(self, msg):
        self.pose=msg
        

    def waypoints_cb(self, waypoints):

        self.base_lane=waypoints # all the existing waypoints
        if not self.waypoints_2d:
            self.waypoints_2d=[[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree= KDTree(self.waypoints_2d)
      

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message. 
        self.stopline_wp_idx = msg.data
        #rospy.logwarn("------------------>stopline_wp_idx: " + str(self.stopline_wp_idx) )


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):# wp1,wp2 are waypoints indexes, not waypoints per se

                                            # return the culmulative distance between each waypoints from wp1 to wp2
                                            # !!! return 0 when wp1>wp2
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            # sum of all the segments length from between 2 waypoints 
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i 
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')





