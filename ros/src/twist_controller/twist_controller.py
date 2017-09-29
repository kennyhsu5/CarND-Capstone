
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


class Controller(object):
    def __init__(self, *args, **kwargs):
        wheel_base = *args[0]
        steer_ratio = *args[1]
        min_speed = *args[2]
        max_lat_accel = *args[3]
        max_steer_angle = *args[4]
        decel_limit = *args[5]
        accel_limit = *args[6]
        vehicle_mass = *args[7]
        wheel_radius = *args[8]

        self.brake_deadband = *args[9]
        self.sample_time = *args[10]

        kp = 1.
        ki = 1.
        kd = 1.
        mn = -decel_limit
        mx = accel_limit

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.velocity_controller = PID(kp, ki, kd, mn, mx)
        self.throttle_filter = LowPassFilter(5*self.sample_time, self.sample_time)

        self.brake_conversion = vehicle_mass*wheel_radius



    def control(self, desired_lin_vel=0., desired_ang_vel=0., current_lin_vel=0., current_ang_vel=0., dbw_enabled=False, sample_time = 0.02):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        steer = self.yawcontroller.get_steering(desired_lin_vel, desired_ang_vel, current_lin_vel)

        velocity_error = desired_lin_vel - current_lin_vel
        
        if dbw_enabled:
        	acc = self.velocity_controller.step(velocity_error, self.sample_time)
        	filtered_acc = self.throttle_filter.filter(acc)

        	if filtered_acc<0:
        		throttle = 0
        		brake = -filtered_acc

        		if abs(brake) < self.brake_deadband:
        			brake = brake + self.brake_deadband 
        	else:
        		throttle = filtered_acc
        		brake = 0
        	
        	return throttle, brake*self.brake_conversion, steer 	

       	else:
       		self.velocity_controller.reset()
       		return 1., 0., 0.



        
