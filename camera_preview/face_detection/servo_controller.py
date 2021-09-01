import numpy as np
import pigpio
import asyncio
from asyncio import Event
import time

class ServoControl:
    def __init__(self):
        self.pwmFrequency = 50
        self.servoGpioPin1 = 13
        self.servoGpioPin2 = 12

        self.maxYawAngle = 60.0
        self.maxPitchAngle = 45.0

        self.halfYawAngle = maxYawAngle / 2
        self.halfPitchAngle = maxPitchAngle / 2

        self.scan_positions = {
            "1": (halfYawAngle, -halfPitchAngle),
            "2": (-halfYawAngle, -halfPitchAngle),
            "3": (halfYawAngle, halfPitchAngle),
            "4": (-halfYawAngle, halfPitchAngle)
        }

        self.scan_transitions = {
            "1": ["2"],
            "2": ["3"],
            "3": ["4"],
            "4": ["1"]
        }

        self.scan_position = "1"
        
        self.currentYawAngle: int = self.scan_positions[self.scan_position][0] 
        self.currentPitchAngle: int = self.scan_positions[self.scan_position][1]

        self.scan_cancellation_event = Event()

        global pwm
        pwm = pigpio.pi()
        pwm.set_mode(self.servoGpioPin1, pigpio.OUTPUT)
        pwm.set_PWM_frequency(self.servoGpioPin1, 330)

        pwm.set_mode(self.servoGpioPin2, pigpio.OUTPUT)
        pwm.set_PWM_frequency(self.servoGpioPin2, 330)

    def moveYawDegrees(self, degrees: int):
        self.currentYawAngle += degrees

        if (self.currentYawAngle > self.maxYawAngle):
            self.currentYawAngle = self.maxYawAngle
        elif (self.currentYawAngle < -self.maxYawAngle):
            self.currentYawAngle = -self.maxYawAngle
            
        pulseWidth = 1000 / 90 * self.currentYawAngle + 1500

        pwm.set_servo_pulsewidth(self.servoGpioPin1, pulseWidth)

    def movePitchDegrees(self, degrees: int):
        self.currentPitchAngle += degrees

        if (self.currentPitchAngle > self.maxPitchAngle):
            self.currentPitchAngle = self.maxPitchAngle
        elif (self.currentPitchAngle < -self.maxPitchAngle):
            self.currentPitchAngle = -self.maxPitchAngle
            
        pulseWidth = 1000 / 90 * self.currentPitchAngle + 1500

        pwm.set_servo_pulsewidth(self.servoGpioPin2, pulseWidth)

    def scan(self, scan_velocity):
        if self.cancellation_event.is_set():
            return
        
        closest_position = "1"
        closest_distance = 10e+8

        for position in self.scan_positions:
            distance = pow(self.currentYawAngle - self.scan.positions[position][0], 2) + pow(self.currentPitchAngle - self.scan.positions[position][1], 2)

            if distance <= closest_distance:
                closest_distance = distance
                closest_position = position

        self.scan_position = closest_position
        initial_d = True

        prev_time = time.time()

        while (not cancellation_event.is_set()):
            if not initial_d:
                self.scan_position = self.scan_transitions[self.scan_position]

            diff_vector = (self.currentYawAngle - self.scan_positions[self.scan_position][0], self.currentPitchAngle - self.scan_positions[self.scan_position][1])

            while diff_vector != (0, 0):
                current_time = time.time()

                if diff_vector[0] < 0:
                    self.moveYawDegrees(-scan_velocity * (current_time - prev_time))
                else:
                    self.moveYawDegrees(scan_velocity * (current_time - prev_time))

                if diff_vector[1] < 0:
                    self.movePitchDegrees(-scan_velocity * (current_time - prev_time))
                else:
                    self.movePitchDegrees(scan_velocity * (current_time - prev_time))
                
                prev_time = time.time()

                diff_vector = (self.currentYawAngle - self.scan_positions[self.scan_position][0], self.currentPitchAngle - self.scan_positions[self.scan_position][1])

            initial_d = False

        self.cancellation_event.clear()
            
    def cancelScan():
        self.cancellation_event.set()

    def destroy(self):
        pwm.set_PWM_dutycycle(self.servoGpioPin1, 0)
        pwm.set_PWM_frequency(self.servoGpioPin1, 0)
        pwm.set_PWM_dutycycle(self.servoGpioPin2, 0)
        pwm.set_PWM_frequency(self.servoGpioPin2, 0)