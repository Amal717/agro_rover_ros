#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO
import time

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class MotorController:
    def __init__(self, en, in1, in2):
        self.en = en
        self.in1 = in1
        self.in2 = in2

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.en, GPIO.OUT)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)

        self.pwm = GPIO.PWM(self.en, 100)  # Set PWM frequency to 100 Hz
        self.pwm.start(0)  # Initialize PWM with duty cycle 0
        self.speed = 0  # Initialize speed attribute

    def set_speed(self, speed):
        self.speed = speed
        self.pwm.ChangeDutyCycle(speed)

    def move_forward(self, speed):
        self.set_speed(speed)
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

    def move_backward(self, speed):
        self.set_speed(speed)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)

    def stop(self):
        self.set_speed(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.speed = 0

class Rover:
    def __init__(self, motorA_pins, motorB_pins):
        enA, in1A, in2A = motorA_pins
        enB, in3B, in4B = motorB_pins
        self.left_motor = MotorController(enA, in1A, in2A)
        self.right_motor = MotorController(enB, in3B, in4B)

    def move(self, linear_velocity, angular_velocity):
        if linear_velocity != 0:
            forward_speed = map_value(linear_velocity, 0, 100, 0, 100)
            if linear_velocity > 0:
                self.left_motor.move_forward(forward_speed)
                self.right_motor.move_forward(forward_speed)
            else:
                self.left_motor.move_backward(abs(forward_speed))
                self.right_motor.move_backward(abs(forward_speed))
        elif angular_velocity != 0:
            turn_speed = map_value(abs(angular_velocity), 0, 100, 0, 100)
            if angular_velocity > 0:
                self.left_motor.move_forward(turn_speed)
                self.right_motor.move_backward(turn_speed)
            else:
                self.left_motor.move_backward(turn_speed)
                self.right_motor.move_forward(turn_speed)
        else:
            self.left_motor.stop()
            self.right_motor.stop()

class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_publisher')
        self.rover = Rover(
            motorA_pins=(17, 18, 27),
            motorB_pins=(22, 23, 24)
        )
        rospy.Subscriber('axis_values', Int32MultiArray, self.axis_callback)
        rospy.spin()

    def axis_callback(self, msg):
        if len(msg.data) >= 2:
            x_axis_value = msg.data[2]
            y_axis_value = msg.data[1]
            linear_velocity = map_value(y_axis_value, 0, 100, -100, 100)
            angular_velocity = map_value(x_axis_value, 0, 100, -100, 100)
            self.rover.move(linear_velocity, angular_velocity)

if __name__ == '__main__':
    try:
        joystick_controller = JoystickController()
    finally:
        GPIO.cleanup()
