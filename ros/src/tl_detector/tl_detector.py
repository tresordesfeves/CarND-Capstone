#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree

import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.waypoints_2d = None


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):  

        self.base_waypoints=waypoints       
        if not self.waypoints_2d:       
            self.waypoints_2d=[[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]       
            self.waypoint_tree= KDTree(self.waypoints_2d)       



    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """


        self.has_image = True # to ignore non visible light ahead 
        self.camera_image = msg


        light_wp, state = self.process_traffic_lights()


        #debug
        rospy.logwarn("light_wp: " + str(light_wp) + ", State: " + str(state) )


        #light_wp: the index of the closest waypoint to the traffic light stop line  ( a line marking on US roadways (in the USA the light is located behing the crossing))
        #light_wp : index within the entire waypoint list ( list is passed in "waypoints_cb")

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    

    def get_closest_waypoint(self, pose):
        
        x,y=pose

        # querry the KDTree with the vehicle current position to find the closest waypoint index
        closest_idx=self.waypoint_tree.query([x,y],1)[1] # ,1 : the closest one, [1], the index in waypoints_2d list
        
        return closest_idx




    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        """if(not self.has_image):
                                    self.prev_light_loc = None
                                    return False
                        
                                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                        
                                #Get classification
                                return self.light_classifier.get_classification(cv_image)
        """
        
        return light.state # not using the classifier but the results from lights come in from the simulator with the state already attached 


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming STOP LINE for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        closest_light_ahead = None  # the index of the neighbor waypoint to the closest light ahead of the vehicle 
        stop_line_wp_idx = None     # index of the neighbor waypoint to the the closest traffic light ahead
         


        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions'] #(x,y coordinates)
        
        if(self.pose): 

            car_pose = (self.pose.pose.position.x, self.pose.pose.position.y)
            car_wp_idx = self.get_closest_waypoint(car_pose) #  index of the neighbor waypoint to the vehicle

            #rospy.logwarn("car_wp_idx: " + str(car_wp_idx) )


             # find the closest visible traffic light (if one exists)

            distance_in_waypoints= len(self.base_waypoints.waypoints) # number of waypoints separating 2 locations # initialized to the maximum value : total number of waypoints

            for i, stop_line in enumerate ( stop_line_positions ): # going through the stop lines one by one to find the closest stop_line ahead of the car 
               stop_line_pose = (stop_line[0],stop_line[1])
               stop_line_wp_idx=self.get_closest_waypoint(stop_line_pose)
               candidate_distance=stop_line_wp_idx - car_wp_idx # distance (number of waypoints) between the car and the stop line closest waypoints
               if candidate_distance >0 : # light to consider because the corresponding  stop line is ahead of the car 
               #   on a side note:  a car will rightfully ignore a light once it  passes its corresponding stop line ( car already in the middle of the crossing)
                   distance_in_waypoints=min( distance_in_waypoints, candidate_distance)
                   
                   closest_light_ahead= self.lights[i] # so far the best candidate


        if closest_light_ahead: # the closest light for which the stop line is ahead of the car was found 

            state = self.get_light_state(closest_light_ahead)

            return stop_line_wp_idx, state # "location" (ie index) an state (Red, Orange, Greem ) : noise from classification and controls are addressed in other nodes 

        return -1, TrafficLight.UNKNOWN # detection failed or no visible light


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
