"""TelloController controller."""
from controller import *
import numpy as np
from simple_pid import PID
from Constants import *


def clamp(value, low, high):
    if value < low:
        return low
    elif value > high:
        return high
    else:
        return value


def sign(x):
    if x > 0:
        return 1
    return -1


class TelloWebotsController:

    def __init__(self):
        self.robot = Robot()
        self.robot.keyboard.enable(TIME_STEP)
        self.keyboard = Keyboard()

        self.gps = GPS("gps")
        self.imu = InertialUnit("inertial unit")
        self.compass = Compass("compass")
        self.gyro = Gyro("gyro")
        self._enable_devices()

        self.target_altitude = target_altitude
        self.back_left_motor = Motor('rear left propeller')
        self.back_right_motor = Motor('rear right propeller')
        self.front_left_motor = Motor('front left propeller')
        self.front_right_motor = Motor('front right propeller')

        self.pitchPID = PID(pitch_Kp, pitch_Ki, pitch_Kd, setpoint=0.0)
        self.rollPID = PID(roll_Kp, roll_Ki, roll_Kd, setpoint=0.0)
        self.throttlePID = PID(throttle_Kp, throttle_Ki, throttle_Kd, setpoint=self.target_altitude)
        self.yawPID = PID(yaw_Kp, yaw_Ki, yaw_Kd, setpoint=yaw_setpoint)
        self.targetX, self.targetY, self.altitude_attained = 0.0, 0.0, False

        self._motors_on()

    def _enable_devices(self):
        self.imu.enable(TIME_STEP)
        self.compass.enable(TIME_STEP)
        self.gyro.enable(TIME_STEP)
        self.gps.enable(TIME_STEP)

    def _motors_on(self):
        motors = [self.back_left_motor, self.back_right_motor, self.front_left_motor, self.front_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(TAKEOFF_THRESHOLD_VELOCITY)

    def run(self):
        while 1:
            self.robot.step(TIME_STEP)
            roll, pitch, _ = self.imu.getRollPitchYaw()
            yaw = self.compass.getValues()[1]
            roll = roll + np.pi/2

            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            y_gps, z_gps, x_gps = self.gps.getValues()

            vertical_input = self.throttlePID(z_gps)
            yaw_input = self.yawPID(yaw)

            if z_gps > self.target_altitude * altitude_attainment_factor:
                self.altitude_attained = True

            self.get_user_input()
            if not self.altitude_attained:
                self.targetX = -1.0
                self.targetY = -1.0

            self.rollPID.setpoint = self.targetX
            self.pitchPID.setpoint = -self.targetY

            print(z_gps)
            roll_input = k_roll_p * roll + roll_acceleration + self.rollPID(-x_gps)
            pitch_input = k_pitch_p * pitch - pitch_acceleration + self.pitchPID(y_gps)

            front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
            front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
            rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
            rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input

            self.back_left_motor.setVelocity(rear_left_motor_input)
            self.back_right_motor.setVelocity(rear_right_motor_input)
            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(front_right_motor_input)
            # self.roll_disturbance, self.pitch_disturbance, self.yaw_disturbance = 0.0, 0.0, 0.0
            #print(k_vertical_thrust, vertical_input , roll_input, pitch_input,  yaw_input)
            #print(self.back_right_motor.getVelocity(), self.back_left_motor.getVelocity(), self.front_right_motor.getVelocity(), self.front_left_motor.getVelocity())

    def get_user_input(self):
        key = self.keyboard.getKey()
        if key == self.keyboard.UP:
            self.target_altitude = 2.0
        elif key == self.keyboard.DOWN:
            self.pitch_disturbance = -2.0
        elif key == self.keyboard.RIGHT:
            self.roll_disturbance = -0.1
        elif key == self.keyboard.LEFT:
            self.roll_disturbance = 0.1





TelloWebotsController().run()