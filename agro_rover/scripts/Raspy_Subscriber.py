#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO
import time

class MotorController:
    def __init__(self, en, in1, in2):
        self.en = en
        self.in1 = in1
        self.in2 = in2

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.en, GPIO.OUT)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)

        self.pwm = GPIO.PWM(self.en, 1000)  # 1000 Hz frequency
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
        self.pwm.ChangeDutyCycle(0)
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.speed = 0

class Rover:
    def __init__(self, motorA_pins, motorB_pins):
        enA, in1A, in2A = motorA_pins
        enB, in3B, in4B = motorB_pins
        self.left_motor = MotorController(enA, in1A, in2A)
        self.right_motor = MotorController(enB, in3B, in4B)

    def move_forward(self, linear_velocity):
        self.left_motor.move_forward(linear_velocity)
        self.right_motor.move_forward(linear_velocity)

    def move_backward(self, linear_velocity):
        self.left_motor.move_backward(linear_velocity)
        self.right_motor.move_backward(linear_velocity)

    def move_left(self, angular_velocity):
        x_left = angular_velocity / 2
        self.left_motor.move_forward(50 - x_left)
        self.right_motor.move_forward(50 + x_left)

    def move_right(self, angular_velocity):
        x_right = angular_velocity / 2
        self.left_motor.move_forward(50 + x_right)
        self.right_motor.move_forward(50 - x_right)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class RaspySubscriber:
    def __init__(self):
        # Initialize the ROS node with the name 'raspy_subscriber'
        rospy.init_node('raspy_subscriber')

        # Create an instance of the Rover class
        self.rover = Rover(
            motorA_pins=(17, 18, 27),
            motorB_pins=(22, 23, 24)
        )

        # Subscribe to the 'joystick_twist' topic with the twist_callback as the callback function
        rospy.Subscriber('axis_values', Int32MultiArray, self.joystick_values_callback)

        # Enter the ROS event loop and wait for callbacks
        rospy.spin()

    def joystick_values_callback(self, msg):
        if len(msg.data) >= 4:
            # Extract x_axis_value and y_axis_value from the received Int32MultiArray
            x_axis_value = msg.data[0]
            y_axis_value = msg.data[3]

            # Map the linear and angular velocities from the joystick values to a range of -100 to 100
            linear_velocity = map_value(y_axis_value, 0, 100, -100, 100)
            angular_velocity = map_value(x_axis_value, 0, 100, -100, 100)

            # Control rover movement based on the mapped velocities
            if linear_velocity > 0:
                forward_speed = map_value(linear_velocity, 0, 100, 0, 100)
                self.rover.move_forward(forward_speed)
            elif linear_velocity < 0:
                backward_speed = map_value(linear_velocity, -100, 0, 0, 100)
                self.rover.move_backward(backward_speed)
            else:
                self.rover.stop()

            if angular_velocity > 0:
                right_turn_speed = map_value(angular_velocity, 0, 100, 0, 100)
                self.rover.move_right(right_turn_speed)
            elif angular_velocity < 0:
                left_turn_speed = map_value(angular_velocity, -100, 0, 0, 100)
                self.rover.move_left(left_turn_speed)
            else:
                self.rover.stop()

if __name__ == '__main__':
    try:
        # Create an instance of the RaspySubscriber class when the script is run
        raspy_subscriber = RaspySubscriber()
    finally:
        GPIO.cleanup()
