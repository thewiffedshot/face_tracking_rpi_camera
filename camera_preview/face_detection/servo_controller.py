import numpy as np
import pigpio

class ServoControl:
    pwmFrequency = 50
    servoGpioPin1 = 13
    servoGpioPin2 = 12
    currentYawAngle: int = 0 
    currentPitchAngle: int = 0 

    maxYawAngle = 60
    maxPitchAngle = 45

    def __init__():
        global pwm
        pwm = pigpio.pi()
        pwm.set_mode(ServoControl.servoGpioPin1, pigpio.OUTPUT)
        pwm.set_PWM_frequency(ServoControl.servoGpioPin1, 330)

        pwm.set_mode(ServoControl.servoGpioPin2, pigpio.OUTPUT)
        pwm.set_PWM_frequency(ServoControl.servoGpioPin2, 330)

    def moveYawDegrees(degrees: int):
        ServoControl.currentYawAngle += degrees

        if (ServoControl.currentYawAngle > ServoControl.maxYawAngle):
            ServoControl.currentYawAngle = ServoControl.maxYawAngle
        elif (ServoControl.currentYawAngle < -ServoControl.maxYawAngle):
            ServoControl.currentYawAngle = -ServoControl.maxYawAngle
            

        pulseWidth = 1000 / 90 * ServoControl.currentYawAngle + 1500

        pwm.set_servo_pulsewidth(ServoControl.servoGpioPin1, pulseWidth)

    def movePitchDegrees(degrees: int):
        ServoControl.currentPitchAngle += degrees

        if (ServoControl.currentPitchAngle > ServoControl.maxPitchAngle):
            ServoControl.currentPitchAngle = ServoControl.maxPitchAngle
        elif (ServoControl.currentPitchAngle < -ServoControl.maxPitchAngle):
            ServoControl.currentPitchAngle = -ServoControl.maxPitchAngle
            

        pulseWidth = 1000 / 90 * ServoControl.currentPitchAngle + 1500

        pwm.set_servo_pulsewidth(ServoControl.servoGpioPin2, pulseWidth)

    def destroy():
        pwm.set_PWM_dutycycle(ServoControl.servoGpioPin1, 0)
        pwm.set_PWM_frequency(ServoControl.servoGpioPin1, 0)
        pwm.set_PWM_dutycycle(ServoControl.servoGpioPin2, 0)
        pwm.set_PWM_frequency(ServoControl.servoGpioPin2, 0)