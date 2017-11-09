#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
import math

from twist_controller import Controller
from yaw_controller import YawController
from lowpass import LowPassFilter

OP_NAME = "dbw_node"

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        self.sampling_rate = 50.0 # 50Hz

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -22.)
        accel_limit = rospy.get_param('~accel_limit', 11.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        rospy.Subscriber('/twist_cmd', TwistStamped, self.movement_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb)

        # hold movement commants from /twist_cmd
        self.movement_command = TwistStamped()
        self.movement_command.header.stamp.secs = 0

        # keep track of velocity
        self.velocity = TwistStamped()
        self.velocity.header.stamp.secs = 0

        # create pid controller for steering angle
        # tau_p, tau_i, tau_d, min_steer_value, max_steer_value
        arg_list_to_controller = {
            'max_lat_accel': max_lat_accel,
            'max_steer_angle': max_steer_angle,
            'steer_ratio': steer_ratio,
            'wheel_base': wheel_base,
            'accel_limit': accel_limit,
            'decel_limit': decel_limit,
            'brake_deadband': brake_deadband,
            'wheel_radius': wheel_radius,
            'sampling_rate': self.sampling_rate,
            'vehicle_mass':vehicle_mass,
            'fuel_capacity':fuel_capacity
        }
        self.twist_controller = Controller(**arg_list_to_controller)
        

        # create lowpass filter
        self.lowpass_filter = LowPassFilter(3, 1)
        self.prev_linear_x_cmd = 0
        self.dbw_enabled = False

        self.loop()
        
    def loop(self):
        rate = rospy.Rate(self.sampling_rate) # 50Hz
        ind = 0
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            
            if self.dbw_enabled is True:
                throttle, brake, steering = self.twist_controller.control(self.movement_command.twist.linear.x,
                                                                self.movement_command.twist.angular.z,
                                                                self.velocity.twist.linear.x,
                                                                self.velocity.twist.angular.z)            
            
            
                self.publish(throttle, brake, steering)
            else:
                self.twist_controller.reset()
                
                
            rate.sleep()

    def dbw_cb(self, msg):
        self.dbw_enabled = msg.data
        rospy.loginfo("dbw_enabled %s", self.dbw_enabled);
        
    def velocity_cb(self, msg):
        self.velocity = msg
        self.twist_controller.control(self.movement_command.twist.linear.x,
                                                                self.movement_command.twist.angular.z,
                                                                self.velocity.twist.linear.x,
                                                                self.velocity.twist.angular.z)
        #rospy.loginfo('current velocity %s', msg)


    def movement_cb(self, msg):
        self.movement_command = msg
        self.twist_controller.control(self.movement_command.twist.linear.x,
                                                                self.movement_command.twist.angular.z,
                                                                self.velocity.twist.linear.x,
                                                                self.velocity.twist.angular.z)
        if (self.movement_command.twist.linear.x != self.prev_linear_x_cmd):
            self.prev_linear_x_cmd = self.movement_command.twist.linear.x
            #rospy.loginfo('mov cmd linearX change %s', msg)
        
    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
