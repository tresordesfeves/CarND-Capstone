from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass,fuel_capacity,brake_deadband,decel_limit,accel_limit, wheel_radius,wheel_base,steer_ratio, max_lat_accel, max_steer_angle):

    	

    	self.yaw_controller = YawController(wheel_base,steer_ratio,0.1,max_lat_accel,max_steer_angle) # instantiating   



    	#setting up the PID controller k coefficients:
    	kp = 0.3
    	kd = 0.0
    	ki = 0.1

    	# setting up safe guards values for the throttle ourput of the PID controller 
    	mn= 0 #minimum throttle
    	mx = 0.2 #maximum throttle

    	self.throttle_controller = PID(kp,ki,kd,mn,mx) # instantiating the controller 

    	# setting up the low-pass filter parameters ( filter to remove unwanted HF noise in the velocity)
    	tau= 0.5 # for calculation of the HF cutoff
    	ts= 0.02 # sampling time 

    	self.vel_lpf=LowPassFilter(tau,ts) # filtered velocity


    	self.vehicle_mass = vehicle_mass
    	self.brake_deadband = brake_deadband
    	self.decel_limit=decel_limit
    	self.accel_limit = accel_limit
    	self.wheel_radius = wheel_radius

    	self.last_time =rospy.get_time(  )


        # TODO: Implement
        pass

    def control(self, current_vel, dbw_enabled, linear_velocity, angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled: # return to manual driving
        	self.throttle_controller.reset() # disconnect the throttle PID controller and reset to zero the accumulated (intgral) error
        	return 0,0,0 # switch back to manual driving

        current_velocity=self.vel_lpf.filt(current_vel) # HF noise clean up


        steering=self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

       	# getting all the parameters ready for the throttle PID controller
       	vel_error= linear_velocity - current_velocity
       	self.last_vel= current_velocity
       	current_time = rospy.get_time()
       	sample_time = current_time - self.last_time
       	self.last_time = current_time

       	throttle=self.throttle_controller.step(vel_error,sample_time) # get the throttle value from the PID throttle controller
       	brake=0

       	if linear_velocity==0 and current_velocity <0.1 :#idling at a stop sign or at a red light: automatic transmission , brake to immobilized vehicle
       		throttle=0
       		brake= 400 # immobilization torque 
       	
       	elif throttle<0.1 and vel_error<0: # not "throttling" anymore and vehicle going faster than intended : it's time to push on the break 

       		throttle = 0 # zero out residual throttle

       		decel = min (abs(vel_error),abs(self.decel_limit) )# these are negative: easier to comprehend in abs values 
       		brake = decel* self.vehicle_mass* self.wheel_radius # torque formula 

       	return throttle,steering ,brake











