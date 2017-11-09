
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

# PID parameters for speed control
KP_SPEED = 1.00
KI_SPEED = 0.0001
KD_SPEED = 0.4

MAX_SPEED = 11.1 # m/sec
MIN_SPEED = 0.0

class Controller(object):
    def __init__(self, **kwargs):
        
    	self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.steer_ratio = kwargs['steer_ratio']
        self.wheel_base = kwargs['wheel_base']
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.brake_deadband = kwargs['brake_deadband']
        self.wheel_radius = kwargs['wheel_radius']
        self.sampling_rate = kwargs['sampling_rate']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']

        self.brake_constant = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius
        
        # outputs throttle/brake command
        self.pid_speed_controller = PID(KP_SPEED, KI_SPEED, KD_SPEED)
        
        
        # outputs steer command
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, MIN_SPEED, self.max_lat_accel, self.max_steer_angle)

    def reset(self):
        self.pid_speed_controller.reset()
    
    
    # just two params right now that correspond to the pid controller.
    def control(self, twist_linear, twist_angular, current_linear, current_angular):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        linear_error = twist_linear - current_linear
        if current_linear < 1.0:
        	self.pid_speed_controller.reset()
        
        acc = self.pid_speed_controller.step(linear_error, self.sampling_rate)
        
        
        if linear_error >= 0.0:
            throttle = acc/MAX_SPEED
            brake = 0
        else:
            throttle = 0.0
            brake = self.brake_constant * current_linear * current_linear * linear_error * linear_error * (MAX_SPEED-twist_linear)  * (MAX_SPEED-twist_linear)
            
            if brake < self.brake_deadband:
                brake = 0.0


        steering = self.yaw_controller.get_steering(twist_linear, twist_angular, current_linear)

        return throttle, brake, steering

	
