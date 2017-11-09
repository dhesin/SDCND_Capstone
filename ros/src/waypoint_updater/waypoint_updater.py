#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
import time
from std_msgs.msg import Int32
import math

'''

	rospy.loginfo(myString) will print to /rosout and is useful for debugging.
	/rosout contains messages from all active nodes and processes.
	To filter on the messages you want, use grep and pass in the string OP_NAME.

	Make sure to include OP_NAME at the beginning of all your rospy.loginfo() messages.

	ex.

	rospy.loginfo(OP_NAME + " -- test messages")

'''
OP_NAME = "waypoint_updater"

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
MAX_DECEL = 5
MAX_ACCEL = 11

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        
        # keep track of which waypoints the car is closest.
        self.cur_pos_wp_ind = None
        self.last_update_ind = None
        self.traffic_waypoint = -2
        self.current_velocity = None

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.waypoints = None
        self.waypoints_set = False;
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        self.updated_waypoints = None

        # TODO: Add other member variables you need below

        rospy.spin()

    def get_closest_waypoint(self, pose_x, pose_y, waypoints):
 
        closest_distance = 100000.0
        closest_point = 0

        for i in range(len(waypoints)):
            # extract waypoint x,y
            wp_x = waypoints[i].pose.pose.position.x
            wp_y = waypoints[i].pose.pose.position.y
            # compute distance from car x,y
            distance = math.sqrt((wp_x - pose_x) ** 2 + (wp_y - pose_y) ** 2)
            # is this point closer than others found so far
            if distance < closest_distance:
                closest_distance = distance
                closest_point = i

        # return closest point found
        return closest_point

    def velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
        
        
    def pose_cb(self, msg):
        
        self.current_position = msg

        if self.waypoints is None:
            return


        self.cur_pos_wp_ind  = self.get_closest_waypoint(self.current_position.pose.position.x, 
            self.current_position.pose.position.y, self.waypoints.waypoints)

        if self.last_update_ind is not None and self.cur_pos_wp_ind > self.last_update_ind:
            wps_since_last_update = self.cur_pos_wp_ind-self.last_update_ind
        else:
            wps_since_last_update = 100

        if (wps_since_last_update<1):
            return

        self.last_update_ind = self.cur_pos_wp_ind
        # create Lane variable
        updated_waypoints = Lane()

        # populate header values
        updated_waypoints.header.seq = self.current_position.header.seq
        updated_waypoints.header.stamp = rospy.Time(0)
        updated_waypoints.header.frame_id = self.current_position.header.frame_id

        # populate waypoints
        if self.cur_pos_wp_ind+LOOKAHEAD_WPS >= self.num_waypoints:
            updated_waypoints.waypoints = self.waypoints.waypoints[self.cur_pos_wp_ind:self.num_waypoints]
            remaining_wps = LOOKAHEAD_WPS-(self.num_waypoints-self.cur_pos_wp_ind)
            numpy.append(updated_waypoints.waypoints, self.waypoints.waypoints[:remaining_wps])
        else:
            updated_waypoints.waypoints = self.waypoints.waypoints[self.cur_pos_wp_ind:self.cur_pos_wp_ind+LOOKAHEAD_WPS]

        # publish updated waypoints list
        self.final_waypoints_pub.publish(updated_waypoints)
        #rospy.loginfo('puplished updated waypoints')
        #rospy.loginfo(updated_waypoints)
        

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.num_waypoints = len(waypoints.waypoints)
        self.waypoints_set = True;

    def decelerate(self, stop_index):

        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        wp1 = self.cur_pos_wp_ind
        wp2 = stop_index
        
        dist_2 = self.distance(self.waypoints.waypoints, wp1, wp2)
        #current_velocity = self.waypoints.waypoints[wp1].twist.twist.linear.x
        a_required = (1.5*self.current_velocity*self.current_velocity)/(dist_2+0.001)    
        #rospy.loginfo("required decellaration:%s", a_required)
        # cannot decelerate, not enough space probably...
        
        for i in range(wp2, wp1-1, -1):
            dist = dist + dl(self.waypoints.waypoints[wp2].pose.pose.position, self.waypoints.waypoints[i].pose.pose.position)
            #vel = math.sqrt(2 * MAX_DECEL * dist)
            vel = math.sqrt(2 * a_required * dist)
            if vel < 2.5:
                vel = 0
            self.waypoints.waypoints[i].twist.twist.linear.x = min(vel, self.waypoints.waypoints[i].twist.twist.linear.x)
            wp2 = i
            #rospy.loginfo("wp index %d speed adjusted to %f", i, self.waypoints.waypoints[i].twist.twist.linear.x)
        #self.waypoints.waypoints[wp2].twist.twist.linear.x = 0.0
        
        return self.cur_pos_wp_ind

    def accelerate(self, speed):

        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        wp1 = self.cur_pos_wp_ind
        
        for i in range(wp1, len(self.waypoints.waypoints)):
            dist = dist+dl(self.waypoints.waypoints[wp1].pose.pose.position, self.waypoints.waypoints[i].pose.pose.position)
            vel = math.sqrt(2 * MAX_ACCEL * dist) + 11.1
            if vel > speed:
                vel = speed
            self.waypoints.waypoints[i].twist.twist.linear.x = vel
            wp1 = i
            #rospy.loginfo("wp index %d speed adjusted to %f", i, self.waypoints.waypoints[i].twist.twist.linear.x)
        for i in range(0, self.cur_pos_wp_ind):
            self.waypoints.waypoints[i].twist.twist.linear.x = speed

        return self.cur_pos_wp_ind


    def puplish_updated_velocities(self, cur_pos_ind):
         
        # create Lane variable
        updated_waypoints = Lane()

        # populate header values
        updated_waypoints.header.seq = self.current_position.header.seq+1
        updated_waypoints.header.stamp = rospy.Time(0)
        updated_waypoints.header.frame_id = self.current_position.header.frame_id

        # populate waypoints
        if cur_pos_ind+LOOKAHEAD_WPS >= self.num_waypoints:
            updated_waypoints.waypoints = self.waypoints.waypoints[cur_pos_ind:self.num_waypoints]
            remaining_wps = LOOKAHEAD_WPS-(self.num_waypoints-cur_pos_ind)
            numpy.append(updated_waypoints.waypoints, self.waypoints.waypoints[:remaining_wps])
        else:
            updated_waypoints.waypoints = self.waypoints.waypoints[cur_pos_ind:cur_pos_ind+LOOKAHEAD_WPS]

        # publish updated waypoints list
        self.last_update_ind = cur_pos_ind
        self.final_waypoints_pub.publish(updated_waypoints)
        self.updated_waypoints = updated_waypoints
        #rospy.loginfo('puplished updated waypoints')
        #rospy.loginfo(updated_waypoints)
        
    def traffic_cb(self, msg):
        
        rospy.loginfo('msg.data %s', msg.data)

        if (self.cur_pos_wp_ind is None):
            return
        if (self.current_velocity is None):
            return;

        if self.traffic_waypoint > 0 and msg.data is -1:
            self.traffic_waypoint = -1
            cur_pos_wp_ind = self.accelerate(11.1)
            self.puplish_updated_velocities(cur_pos_wp_ind)
            #rospy.loginfo('red OFF %s/%s', self.traffic_waypoint, self.cur_pos_wp_ind)

            
        elif (self.traffic_waypoint is -1 and msg.data > 0) or (self.traffic_waypoint is -2):
            self.traffic_waypoint = msg.data
            cur_pos_wp_ind = self.decelerate(self.traffic_waypoint)
            self.puplish_updated_velocities(cur_pos_wp_ind)
            #rospy.loginfo('red ON %s/%s', self.traffic_waypoint, self.cur_pos_wp_ind)

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

    def decelerate_original(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = self.distance(wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
