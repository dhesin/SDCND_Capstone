#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 1

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.lights_wp_indexes = []


        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']
        print(self.stop_line_positions)
        print("RED", TrafficLight.RED)
        print("YELLOW", TrafficLight.YELLOW)
        print("GREEN", TrafficLight.GREEN)
        print("UNKNOWN", TrafficLight.UNKNOWN)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.light_wp = -1
        self.prev_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        #sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=55000000 )

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        for i in range(len(self.stop_line_positions)):   
            pose = Pose()
            pose.position.x = self.stop_line_positions[i][0]
            pose.position.y = self.stop_line_positions[i][1]
            wp_ind =  self.get_closest_waypoint(pose)  
            self.lights_wp_indexes.append(wp_ind)

        print('light waypoint indexes %s', self.lights_wp_indexes)


    def traffic_cb(self, msg):
        self.lights = msg.lights
        print(self.lights)

        if (len(self.lights_wp_indexes) is 0):
            self.lights_wp_indexes = []

            for i in range(len(self.lights)):   
                light = self.lights[i] 
                wp_ind =  self.get_closest_waypoint(light.pose.pose)  
                self.lights_wp_indexes.append(wp_ind)

            print('light waypoint indexes %s', self.lights_wp_indexes)
        #assert(len(self.lights) == len(self.lights_wp_indexes))

        # light_wp, state = self.process_traffic_lights_from_simulator()

        # '''
        # Publish upcoming red lights at camera frequency.
        # Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        # of times till we start using it. Otherwise the previous stable state is
        # used.
        # '''
        # if self.state != state:
        #     self.state_count = 0
        #     self.state = state
        # elif self.state_count >= STATE_COUNT_THRESHOLD:
        #     self.last_state = self.state
        #     light_wp = light_wp if state == TrafficLight.RED else -1
        #     self.last_wp = light_wp
        #     self.upcoming_red_light_pub.publish(Int32(light_wp))
        # else:
        #     self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        # self.state_count += 1
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        light_wp, new_state = self.process_traffic_lights()
        #print("self.prev_state/self.state/new.state/light_wp", self.prev_state, self.state, new_state, light_wp)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if (self.state is TrafficLight.RED) and (self.light_wp == light_wp) and (new_state is TrafficLight.GREEN):
            self.state = TrafficLight.UNKNOWN
            self.light_wp = -1
            self.upcoming_red_light_pub.publish(Int32(-1))
            print("RED OFF:", light_wp, new_state)
            #rospy.loginfo("RED OFF %s/%s", light_wp, new_state)
        # sometimes car cannot stop on time for RED light. Check if this is the case
        elif (self.state is TrafficLight.RED) and (self.light_wp is not light_wp):
            self.state = TrafficLight.UNKNOWN
            self.light_wp = -1
            self.upcoming_red_light_pub.publish(Int32(-1))
            print("RED OFF:", light_wp, new_state)
            #rospy.loginfo("RED OFF %s/%s", light_wp, new_state)

        elif (self.state is TrafficLight.UNKNOWN) and ((new_state is TrafficLight.RED)):
            self.state = TrafficLight.RED
            self.light_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            print("RED ON:", light_wp, new_state)
            #rospy.loginfo("RED ON %s/%s", light_wp, new_state)
        
 
        
#        if (self.state is TrafficLight.RED) and (self.light_wp == light_wp) and (new_state is TrafficLight.GREEN):
#            self.prev_state = TrafficLight.UNKNOWN
#            self.state = TrafficLight.UNKNOWN
#            self.light_wp = -1
#            self.upcoming_red_light_pub.publish(Int32(-1))
#            print("RED OFF 1:", light_wp, new_state)
#            #rospy.loginfo("RED OFF %s/%s", light_wp, new_state)
#        elif (self.state is TrafficLight.RED) and (self.light_wp == light_wp) and (new_state is TrafficLight.YELLOW):
#            self.prev_state = TrafficLight.RED
#            self.state = TrafficLight.YELLOW
#            self.light_wp = -1
#            self.upcoming_red_light_pub.publish(Int32(-1))
#            print("RED OFF with Yellow:", light_wp, new_state)
#            #rospy.loginfo("RED OFF %s/%s", light_wp, new_state)
#        elif (self.prev_state is TrafficLight.RED) and (self.state is TrafficLight.YELLOW) and ((new_state is TrafficLight.GREEN)):
#            self.prev_state = TrafficLight.UNKNOWN
#            self.state = TrafficLight.UNKNOWN
#            assert(self.light_wp == light_wp)
#            self.light_wp = -1
#            self.upcoming_red_light_pub.publish(Int32(-1))
#            print("RED OFF with green after yellow:", light_wp, new_state)
#            #rospy.loginfo("RED OFF %s/%s", light_wp, new_state)
#        # sometimes car cannot stop on time for RED light. Check if this is the case
#        elif (self.state is TrafficLight.RED) and (self.light_wp is not light_wp):
#            self.prev_state = TrafficLight.UNKNOWN
#            self.state = TrafficLight.UNKNOWN
#            self.light_wp = -1
#            self.upcoming_red_light_pub.publish(Int32(-1))
#            print("RED OFF 2:", light_wp, new_state)
#            #rospy.loginfo("RED OFF %s/%s", light_wp, new_state)
#        # sometimes car cannot stop on time for RED light. Check if this is the case    
#        elif (self.prev_state is TrafficLight.RED) and (self.state is TrafficLight.YELLOW) and (self.light_wp is not light_wp):
#            self.prev_state = TrafficLight.UNKNOWN
#            self.state = TrafficLight.UNKNOWN
#            self.light_wp = -1
#            self.upcoming_red_light_pub.publish(Int32(-1))
#            print("RED OFF 3:", light_wp, new_state)
#            #rospy.loginfo("RED OFF %s/%s", light_wp, new_state)
#        elif (self.state is TrafficLight.UNKNOWN) and ((new_state is TrafficLight.RED) or (new_state is TrafficLight.YELLOW)):
#            self.prev_state = self.state
#            self.state = new_state
#            self.light_wp = light_wp
#            self.upcoming_red_light_pub.publish(Int32(light_wp))
#            print("RED ON:", light_wp, new_state)
#            #rospy.loginfo("RED ON %s/%s", light_wp, new_state)
#        elif (self.prev_state is TrafficLight.UNKNOWN) and (self.state is TrafficLight.YELLOW) and ((new_state is TrafficLight.RED)):
#            self.prev_state = TrafficLight.UNKNOWN
#            self.state = TrafficLight.RED
#            self.light_wp = light_wp
#            self.upcoming_red_light_pub.publish(Int32(light_wp))
#            print("RED ON:", light_wp, new_state)
#            #rospy.loginfo("RED ON %s/%s", light_wp, new_state)

 
 
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        if self.waypoints is None:
            return

        closest_distance = 100000.0
        closest_point = 0

        for i in range(len(self.waypoints.waypoints)):
            # extract waypoint x,y
            wp_x = self.waypoints.waypoints[i].pose.pose.position.x
            wp_y = self.waypoints.waypoints[i].pose.pose.position.y
            # compute distance from car x,y
            distance = math.sqrt((wp_x - pose.position.x) ** 2 + (wp_y - pose.position.y) ** 2)
            # is this point closer than others found so far
            if distance < closest_distance:
                closest_distance = distance
                closest_point = i

        # return closest point found
        return closest_point

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN

        # awoo: we should create a cv2 image using 'rgb8' encoding as that's
        #       what we used for model training
        #       Also, encoding used by the simulator may be different from Carla
        #       However, for easy converstion we should still use bgr8 as this is a cv_image
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # in theory, the image_message should already be in 
        #Get classification
        return self.light_classifier.get_classification(cv_image)


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        if len(self.lights_wp_indexes) is 0:
            return -6, TrafficLight.UNKNOWN;

        if self.pose is None:
            return -7, TrafficLight.UNKNOWN

        car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        ind = 0
        while ind < len(self.lights_wp_indexes) and car_position > self.lights_wp_indexes[ind]:
            ind = ind+1
            
        if ind >= len(self.lights_wp_indexes):
            ind = 0


        #print("IND/LEN/CAR POSITION is %s - %s", ind, len(self.lights_wp_indexes), car_position)
        assert(ind >= 0) 
        assert(ind < len(self.lights_wp_indexes))
        light_wp_ind = self.lights_wp_indexes[ind]
        #assert(light_wp_ind > car_position)

        if light_wp_ind-car_position > 150 or light_wp_ind < car_position:
            #print("light_wp_ind/car_position", light_wp_ind, car_position)
            return light_wp_ind, TrafficLight.UNKNOWN

        light = None
        
        state = self.get_light_state(light)
        #state = light.state
        #print("light_wp_ind/state:%s/%s", light_wp_ind, state)
        return light_wp_ind, state

 
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
