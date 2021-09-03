import numpy as np
import pigpio

import time
from threading import Lock

class ServoControl:
    epsilon = 5e-4

    def __init__(self):
        self.servos_lock = Lock()

        self.pwmFrequency = 50
        self.servoGpioPin1 = 13
        self.servoGpioPin2 = 12

        self.maxYawAngle = 30.0
        self.maxPitchAngle = 23.0

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

    def moveYawDegrees(self, degrees: float):
        self.servos_lock.acquire()
        self.currentYawAngle += degrees

        if (self.currentYawAngle > self.maxYawAngle):
            self.currentYawAngle = self.maxYawAngle
        elif (self.currentYawAngle < -self.maxYawAngle):
            self.currentYawAngle = -self.maxYawAngle
            
        pulseWidth = 1000 / 90 * self.currentYawAngle + 1500

        pwm.set_servo_pulsewidth(self.servoGpioPin1, pulseWidth)
        self.servos_lock.release()

    def movePitchDegrees(self, degrees: float):
        self.servos_lock.acquire()
        self.currentPitchAngle += degrees

        if (self.currentPitchAngle > self.maxPitchAngle):
            self.currentPitchAngle = self.maxPitchAngle
        elif (self.currentPitchAngle < -self.maxPitchAngle):
            self.currentPitchAngle = -self.maxPitchAngle
            
        pulseWidth = 1000 / 90 * self.currentPitchAngle + 1500

        pwm.set_servo_pulsewidth(self.servoGpioPin2, pulseWidth)
        self.servos_lock.release()

    def scan(self, scan_velocity, scan_cancellation_event):
        if scan_cancellation_event.is_set():
            scan_cancellation_event.clear()
            return
        
        closest_position = "1"
        closest_distance = 10e+8

        for position in self.scan_positions:
            distance = pow(self.currentYawAngle - self.scan_positions[position][0], 2) + pow(self.currentPitchAngle - self.scan_positions[position][1], 2)

            if distance <= closest_distance:
                closest_distance = distance
                closest_position = position

        self.scan_position = closest_position
        initial_d = True

        prev_time = 0

        while (not scan_cancellation_event.is_set()):
            if not initial_d:
                self.scan_position = self.scan_transitions[str(self.scan_position)][0]

            print(self.scan_position)
            diff_vector = (self.currentYawAngle - self.scan_positions.get(str(self.scan_position))[0], self.currentPitchAngle - self.scan_positions.get(str(self.scan_position))[1])

            while (abs(diff_vector[0]) > ServoControl.epsilon or abs(diff_vector[1]) > ServoControl.epsilon) and not scan_cancellation_event.is_set():
                current_time = time.time()

                if diff_vector[0] < 0:
                    self.moveYawDegrees(scan_velocity * (current_time - prev_time))
                else:
                    self.moveYawDegrees(-scan_velocity * (current_time - prev_time))

                if diff_vector[1] < 0:
                    self.movePitchDegrees(scan_velocity * (current_time - prev_time))
                else:
                    self.movePitchDegrees(-scan_velocity * (current_time - prev_time))
                    
                prev_time = current_time
                diff_vector = (self.currentYawAngle - self.scan_positions[str(self.scan_position)][0], self.currentPitchAngle - self.scan_positions[str(self.scan_position)][1])

            initial_d = False

        scan_cancellation_event.clear()

    def destroy(self):
        pwm.set_PWM_dutycycle(self.servoGpioPin1, 0)
        pwm.set_PWM_frequency(self.servoGpioPin1, 0)
        pwm.set_PWM_dutycycle(self.servoGpioPin2, 0)
        pwm.set_PWM_frequency(self.servoGpioPin2, 0)