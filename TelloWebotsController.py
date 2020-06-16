"""TelloController controller."""
from controller import *
import numpy as np
from simple_pid import PID
from Constants import *


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

        self.throttlePID = PID(throttle_Kp, throttle_Ki, throttle_Kd, setpoint=self.target_altitude)

        self.robot.step(TIME_STEP)
        self.targetX, self.targetY, self.altitude_attained = self.gps.getValues()[2], self.gps.getValues()[0], False
        self.target_yaw = self.compass.getValues()[1]
        self.pitchPID = PID(pitch_Kp, pitch_Ki, pitch_Kd, setpoint=self.targetY)
        self.rollPID = PID(roll_Kp, roll_Ki, roll_Kd, setpoint=self.targetX)
        self.yawPID = PID(yaw_Kp, yaw_Ki, yaw_Kd, setpoint=self.target_yaw)

        self._motors_on()

    def _enable_devices(self):
        self.imu.enable(TIME_STEP)
        self.compass.enable(TIME_STEP)
        self.gyro.enable(TIME_STEP)
        self.gps.enable(TIME_STEP)

    def _motors_on(self):
        motors = [self.back_left_motor, self.front_left_motor, self.front_right_motor, self.back_right_motor]
        opposite = True
        for motor in motors:
            motor.setPosition(float('inf') if opposite else -float('inf'))
            opposite = not opposite
            motor.setVelocity(TAKEOFF_THRESHOLD_VELOCITY)

    def run(self):
        while 1:
            self.robot.step(TIME_STEP)
            roll, pitch, _ = self.imu.getRollPitchYaw()
            yaw = self.compass.getValues()[1]
            roll = roll + np.pi/2

            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            y_gps, z_gps, x_gps = self.gps.getValues()

            self.get_user_input()
            vertical_input = self.throttlePID(z_gps)

            yaw_input = self.yawPID(yaw)
            roll_input = k_roll_p * roll + roll_acceleration + self.rollPID(x_gps)
            pitch_input = k_pitch_p * pitch - pitch_acceleration - self.pitchPID(y_gps)

            #yaw_input = 0.0
            front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
            front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
            rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
            rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input

            self.back_left_motor.setVelocity(rear_left_motor_input)
            self.back_right_motor.setVelocity(rear_right_motor_input)
            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(front_right_motor_input)

            print(self.back_left_motor.getVelocity(), self.back_right_motor.getVelocity(), self.front_right_motor.getVelocity(), self.front_left_motor.getVelocity())

    def get_user_input(self):
        key = self.keyboard.getKey()
        if key == self.keyboard.UP:
            self.target_altitude = self.target_altitude + 0.02
            self.throttlePID.setpoint = self.target_altitude
        elif key == self.keyboard.DOWN:
            self.target_altitude = self.target_altitude - 0.02
            self.throttlePID.setpoint = self.target_altitude
        elif key == ord('D'):
            self.targetY = self.targetY + 0.01
            self.pitchPID.setpoint = self.targetY
        elif key == ord('U'):
            self.targetY = self.targetY - 0.01
            self.pitchPID.setpoint = self.targetY
        elif key == self.keyboard.LEFT:
            self.targetX = self.targetX + 0.01
            self.rollPID.setpoint = self.targetX
        elif key == self.keyboard.RIGHT:
            self.targetX = self.targetX - 0.01
            self.rollPID.setpoint = self.targetX
        elif key == self.keyboard.SHIFT + self.keyboard.RIGHT:
            self.target_yaw = self.target_yaw + 0.122
            self.yawPID.setpoint = self.target_yaw
        elif key == self.keyboard.SHIFT + self.keyboard.LEFT:
            self.target_yaw = self.target_yaw - 0.122
            self.yawPID.setpoint = self.target_yaw



TelloWebotsController().run()
