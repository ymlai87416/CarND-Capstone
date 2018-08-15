import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        throttle_kp = 0.3  # 1.5 #0.3
        throttle_ki = 0.1  # 2.4 #0.1
        throttle_kd = 0.  # 0.1 #0.

        throttle_mn = 0.  # Minimum throttle value
        throttle_mx = 0.2  # Maximum throttle value

        steering_kp = 0.2  # 0.15
        steering_ki = 0.003
        steering_kd = 0.005

        self.throttle_controller = PID(throttle_kp, throttle_ki, throttle_kd, throttle_mn, throttle_mx)
        self.steering_correct_controller = PID(steering_kp, steering_ki, steering_kd, -max_steer_angle, max_steer_angle)

        kp_steer = .8
        ki_steer = 0.
        kd_steer = 0.05

        mn_steer = -max_steer_angle  # Minimum steer value
        mx_steer = max_steer_angle  # Maximum steer value
        self.steer_controller = PID(kp_steer, ki_steer, kd_steer, mn_steer, mx_steer)

        tau = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts = .02  # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.max_steer_angle = max_steer_angle

        self.last_time = rospy.get_time()

    def control(self, current_vel, linear_vel, angular_vel, cte, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            self.steering_correct_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        # rospy.logwarn("Angular velocity: {0}".format(angular_vel))
        # rospy.logwarn("Target velocity: {0}".format(linear_vel))
        # rospy.logwarn("Current velocity: {0}".format(current_vel))
        # rospy.logwarn("Filtered velocity: {0}".format(self.vel_lpf.get()))

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        feedfoward_steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        corrective_steering = self.steering_correct_controller.step(cte*cte, sample_time)

        steering = feedfoward_steering + corrective_steering
        steering = min(self.max_steer_angle, max(-self.max_steer_angle, steering))
        #steering = self.steer_controller.step(feedfoward_steering, sample_time)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0 and current_vel < 0.1:
            throttle = 0
            brake = 700  # N*m - to hold the car in place if we are stopped at a light. Acceleration ~ 1m/s^2
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius  # Torque N*m

        return throttle, brake, steering