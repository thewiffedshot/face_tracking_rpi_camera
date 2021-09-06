from cv2 import sqrt
import numpy as np
import pigpio
import math

import time
from threading import Lock

class ServoControl:
    epsilon = 5e-4

    def __init__(self):
        self.servos_lock = Lock()

        self.pwmFrequency = 50
        self.servoGpioPin1 = 13
        self.servoGpioPin2 = 12

        self.maxYawAngle = 45.0
        self.maxPitchAngle = 30.0

        self.scan_positions = {
            "1": (-self.maxYawAngle, -self.maxPitchAngle),
            "2": (self.maxYawAngle, -self.maxPitchAngle),
            "3": (self.maxYawAngle, self.maxPitchAngle),
            "4": (-self.maxYawAngle, self.maxPitchAngle)
        }

        self.scan_transitions = {
            "1": ["3"],
            "2": ["1"],
            "3": ["4"],
            "4": ["2"]
        }

        self.scan_position = "1"
        
        self.currentYawAngle: float = self.scan_positions[self.scan_position][0] 
        self.currentPitchAngle: float = self.scan_positions[self.scan_position][1]

        global pwm
        pwm = pigpio.pi()
        pwm.set_mode(self.servoGpioPin1, pigpio.OUTPUT)
        pwm.set_PWM_frequency(self.servoGpioPin1, 330)

        pwm.set_mode(self.servoGpioPin2, pigpio.OUTPUT)
        pwm.set_PWM_frequency(self.servoGpioPin2, 330)

    def moveYawDegrees(self, degrees: float) -> bool:
        self.servos_lock.acquire()
        self.currentYawAngle += degrees

        lock: bool = False

        if (self.currentYawAngle > self.maxYawAngle):
            self.currentYawAngle = self.maxYawAngle
            lock = True
        elif (self.currentYawAngle < -self.maxYawAngle):
            self.currentYawAngle = -self.maxYawAngle
            lock = True
            
        pulseWidth = 1000 / 90 * self.currentYawAngle + 1500

        pwm.set_servo_pulsewidth(self.servoGpioPin1, pulseWidth)
        self.servos_lock.release()

        return lock

    def movePitchDegrees(self, degrees: float) -> bool:
        self.servos_lock.acquire()
        self.currentPitchAngle += degrees

        lock: bool = False

        if (self.currentPitchAngle > self.maxPitchAngle):
            self.currentPitchAngle = self.maxPitchAngle
            lock = True
        elif (self.currentPitchAngle < -self.maxPitchAngle):
            self.currentPitchAngle = -self.maxPitchAngle
            lock = True
            
        pulseWidth = 1000 / 90 * self.currentPitchAngle + 1500

        pwm.set_servo_pulsewidth(self.servoGpioPin2, pulseWidth)
        self.servos_lock.release()

        return lock

    def scan(self, scan_velocity, scan_started_event, scan_cancellation_event):
        scan_started_event.set()
        
        if scan_cancellation_event.is_set():
            scan_cancellation_event.clear()
            return

        print("Started")

        closest_position = "1"
        closest_distance = 10e+8

        for position in self.scan_positions:
            distance = pow(self.currentYawAngle - self.scan_positions[position][0], 2) + pow(self.currentPitchAngle - self.scan_positions[position][1], 2)

            if distance <= closest_distance:
                closest_distance = distance
                closest_position = position

        self.scan_position = closest_position
        initial_d = True

        prev_time = time.time()

        while (not scan_cancellation_event.is_set()):
            if not initial_d:
                self.scan_position = self.scan_transitions[str(self.scan_position)][0]

            diff_vector = (self.currentYawAngle - self.scan_positions.get(str(self.scan_position))[0], self.currentPitchAngle - self.scan_positions.get(str(self.scan_position))[1])
            diff_vector_length: float = math.sqrt(pow(diff_vector[0], 2) + pow(diff_vector[1], 2))
            dir_vector = (.0, .0) if diff_vector == (.0, .0) else tuple((diff_vector[0] / diff_vector_length, diff_vector[1] / diff_vector_length))

            y_lockout: bool = False
            x_lockout: bool = False

            print(self.scan_position)

            while (abs(diff_vector[0]) > ServoControl.epsilon or abs(diff_vector[1]) > ServoControl.epsilon and (not x_lockout or not y_lockout)) and not scan_cancellation_event.is_set():
                current_time = time.time()

                y_lockout = self.moveYawDegrees(dir_vector[0] * -scan_velocity * (current_time - prev_time))
                x_lockout = self.movePitchDegrees(dir_vector[1] * -scan_velocity * (current_time - prev_time))

                prev_time = current_time

                diff_vector = (self.currentYawAngle - self.scan_positions.get(str(self.scan_position))[0], self.currentPitchAngle - self.scan_positions.get(str(self.scan_position))[1])
                diff_vector_length = math.sqrt(pow(diff_vector[0], 2) + pow(diff_vector[1], 2))
                dir_vector = (.0, .0) if diff_vector == (.0, .0) else tuple((diff_vector[0] / diff_vector_length, diff_vector[1] / diff_vector_length))

            initial_d = False

        print("Scan done")

        scan_started_event.clear()

    def destroy(self):
        pwm.set_PWM_dutycycle(self.servoGpioPin1, 0)
        pwm.set_PWM_frequency(self.servoGpioPin1, 0)
        pwm.set_PWM_dutycycle(self.servoGpioPin2, 0)
        pwm.set_PWM_frequency(self.servoGpioPin2, 0)