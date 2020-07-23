"""TelloController controller."""
from controller import *
from simple_pid import PID
from Constants import *
import threading
from utils import *


class TelloWebotsController:

    def __init__(self):
        self.robot = Robot()
        self.robot.keyboard.enable(TIME_STEP)
        self.keyboard = Keyboard()
        self.instructToLand = False

        self.gps = GPS("gps")
        self.imu = InertialUnit("inertial unit")
        self.compass = Compass("compass")
        self.gyro = Gyro("gyro")
        self.height_sensor = DistanceSensor("distance sensor")
        self.camera = Camera("camera")
        self._enable_devices()

        self.target_altitude = takeoff_altitude
        self.back_left_motor = Motor('rear left propeller')
        self.back_right_motor = Motor('rear right propeller')
        self.front_left_motor = Motor('front left propeller')
        self.front_right_motor = Motor('front right propeller')

        self.throttlePID = PID(throttle_Kp, throttle_Ki, throttle_Kd, setpoint=self.target_altitude)
        self.targetX, self.targetY = 0, 0
        self.last_yaw = 0
        self.target_yaw = self.last_yaw
        self.pitchPID = PID(pitch_Kp, pitch_Ki, pitch_Kd, setpoint=self.targetY)
        self.rollPID = PID(roll_Kp, roll_Ki, roll_Kd, setpoint=self.targetX)
        self.yawPID = PID(yaw_Kp, yaw_Ki, yaw_Kd, setpoint=self.target_yaw)
        self.robot.step(TIME_STEP)

    def _enable_devices(self):
        self.imu.enable(TIME_STEP)
        self.compass.enable(TIME_STEP)
        self.gyro.enable(TIME_STEP)
        self.gps.enable(TIME_STEP)
        self.height_sensor.enable(TIME_STEP)
        self.camera.enable(TIME_STEP)

    def _motors_on(self):
        motors = [self.back_left_motor, self.front_left_motor, self.front_right_motor, self.back_right_motor]
        opposite = True
        for motor in motors:
            motor.setPosition(float('inf') if opposite else -float('inf'))
            opposite = not opposite
            motor.setVelocity(TAKEOFF_THRESHOLD_VELOCITY)

    def take_off(self):
        self.robot.step(TIME_STEP)
        self.target_altitude = takeoff_altitude
        self.targetX, self.targetY = self.gps.getValues()[2], self.gps.getValues()[0]
        self.last_yaw = self.imu.getRollPitchYaw()[2] + np.pi
        self.target_yaw = self.last_yaw
        self.pitchPID.setpoint = self.targetY
        self.rollPID.setpoint = self.targetX
        self.yawPID.setpoint = self.target_yaw
        self._motors_on()
        thread = threading.Thread(target=self._run, daemon=False)
        thread.start()


    def _run(self):
        self.instructToLand = False
        while not (self.instructToLand and self.height_sensor.getValue() <= min_sensor_height):
            self.robot.step(TIME_STEP)
            roll, pitch, raw_yaw = self.imu.getRollPitchYaw()
            yaw = self._calc_yaw(raw_yaw + np.pi)
            roll = roll + np.pi/2

            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            y_gps, z_gps, x_gps = self.gps.getValues()
            x_gps, y_gps = normalize_coordinates(x_gps, y_gps, yaw)

            self.get_user_input()
            self._set_PID(yaw)
            vertical_input = self.throttlePID(z_gps)

            yaw_input = self.yawPID(yaw)
            roll_input = k_roll_p * roll + roll_acceleration + self.rollPID(x_gps)
            pitch_input = k_pitch_p * pitch - pitch_acceleration - self.pitchPID(y_gps)

            front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
            front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
            rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
            rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
            self._set_motor_velocities(rear_left_motor_input, rear_right_motor_input, front_left_motor_input, front_right_motor_input)

            self.last_yaw = yaw

        self._set_motor_velocities(0, 0, 0, 0)

    def _set_motor_velocities(self, rear_left_motor_input, rear_right_motor_input, front_left_motor_input, front_right_motor_input):
        self.back_left_motor.setVelocity(rear_left_motor_input)
        self.back_right_motor.setVelocity(rear_right_motor_input)
        self.front_left_motor.setVelocity(front_left_motor_input)
        self.front_right_motor.setVelocity(front_right_motor_input)

    def _calc_yaw(self, raw_yaw):
        last_yaw = np.abs(self.last_yaw - (self.last_yaw // (2 * np.pi)) * 2 * np.pi)
        diff = raw_yaw - last_yaw
        #crossed from 2pi to 0 or 0 to 2pi
        if np.abs(raw_yaw - last_yaw) >= np.pi:
            if raw_yaw > last_yaw:
                diff = -(2*np.pi - raw_yaw + last_yaw)
            else:
                diff = 2 * np.pi - last_yaw + raw_yaw
        return self.last_yaw + diff

    def take_picture(self):
        image = self.camera.getImageArray()
        return np.rot90(np.array(image), k=3)

    def get_user_input(self):
        key = self.keyboard.getKey()
        if key == self.keyboard.UP:
            self.up()
        elif key == self.keyboard.DOWN:
            self.down()
        elif key == ord('D'):
            self.backward()
        elif key == ord('U'):
            self.forward()
        elif key == ord('L'):
            self.land()
        elif key == self.keyboard.LEFT:
            self.left()
        elif key == self.keyboard.RIGHT:
            self.right()
        elif key == self.keyboard.SHIFT + self.keyboard.RIGHT:
            self.cw()
        elif key == self.keyboard.SHIFT + self.keyboard.LEFT:
            self.ccw()

    def _set_PID(self, yaw):
        self.targetX, self.targetY = normalize_coordinates(self.targetX, self.targetY, yaw - self.last_yaw)
        self.throttlePID.setpoint = self.target_altitude
        self.pitchPID.setpoint = self.targetY
        self.rollPID.setpoint = self.targetX
        self.yawPID.setpoint = self.target_yaw

    def land(self):
        z_gps = self.gps.getValues()[1]
        self.target_altitude = z_gps + min_landing_height - self.height_sensor.getValue()
        self.instructToLand = True

    def up(self):
        self.target_altitude = self.target_altitude + z_epsilon

    def down(self):
        self.target_altitude = self.target_altitude - z_epsilon

    def left(self):
        self.targetX = self.targetX + x_epsilon

    def right(self):
        self.targetX = self.targetX - x_epsilon

    def forward(self):
        self.targetY = self.targetY - y_epsilon

    def backward(self):
        self.targetY = self.targetY + y_epsilon

    def cw(self):
        self.target_yaw = self.target_yaw - rotation_epsilon

    def ccw(self):
        self.target_yaw = self.target_yaw + rotation_epsilon


# controller = TelloWebotsController()
# controller.take_off()

