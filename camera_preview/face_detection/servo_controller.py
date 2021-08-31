import numpy as np
import pigpio

class ServoControl:
    def __init__(self):
        self.pwmFrequency = 50
        self.servoGpioPin1 = 13
        self.servoGpioPin2 = 12
        self.currentYawAngle: int = 0 
        self.currentPitchAngle: int = 0 

        self.maxYawAngle = 60
        self.maxPitchAngle = 45

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

    def destroy(self):
        pwm.set_PWM_dutycycle(self.servoGpioPin1, 0)
        pwm.set_PWM_frequency(self.servoGpioPin1, 0)
        pwm.set_PWM_dutycycle(self.servoGpioPin2, 0)
        pwm.set_PWM_frequency(self.servoGpioPin2, 0)