#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
import wiringpi as wp
import time

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class MotorController:
    def __init__(self, en, in1, in2):
        self.en = en
        self.in1 = in1
        self.in2 = in2

        wp.wiringPiSetupGpio()  # Initialize WiringPi with BCM GPIO numbering
        wp.pinMode(self.en, wp.GPIO.PWM_OUTPUT)
        wp.pinMode(self.in1, wp.GPIO.OUTPUT)
        wp.pinMode(self.in2, wp.GPIO.OUTPUT)

        wp.softPwmCreate(self.en, 0, 100)  # Create PWM on the specified pin
        self.speed = 0  # Initialize speed attribute

    def set_speed(self, speed):
        self.speed = speed
        wp.softPwmWrite(self.en, speed)

    def move_forward(self, speed):
        self.set_speed(speed)
        wp.digitalWrite(self.in1, wp.GPIO.HIGH)
        wp.digitalWrite(self.in2, wp.GPIO.LOW)

    def move_backward(self, speed):
        self.set_speed(speed)
        wp.digitalWrite(self.in1, wp.GPIO.LOW)
        wp.digitalWrite(self.in2, wp.GPIO.HIGH)

    def stop(self):
        self.set_speed(0)
        wp.digitalWrite(self.in1, wp.GPIO.LOW)
        wp.digitalWrite(self.in2, wp.GPIO.LOW)
        self.speed = 0

class Rover:
    def __init__(self, motorA_pins, motorB_pins):
        enA, in1A, in2A = motorA_pins
        enB, in3B, in4B = motorB_pins
        self.left_motor = MotorController(enA, in1A, in2A)
        self.right_motor = MotorController(enB, in3B, in4B)

    def move(self, linear_velocity, angular_velocity):
        # Move forward or backward based on the linear_velocity value
        if linear_velocity != 0:
            forward_speed = map_value(linear_velocity, 0, 100, 0, 100)
            if linear_velocity > 0:
                self.left_motor.move_forward(forward_speed)
                self.right_motor.move_forward(forward_speed)
            else:
                self.left_motor.move_backward(abs(forward_speed))
                self.right_motor.move_backward(abs(forward_speed))
        elif angular_velocity != 0:
            # Move right or left based on the angular_velocity value
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
        # Initialize the ROS node with the name 'joystick_controller'
        rospy.init_node('joystick_controller')

        # Create an instance of the Rover class
        self.rover = Rover(
            motorA_pins=(17, 18, 27),
            motorB_pins=(22, 23, 24)
        )

        # Subscribe to the 'axis_values' topic with the axis_callback as the callback function
        rospy.Subscriber('axis_values', Int32MultiArray, self.axis_callback)

        # Enter the ROS event loop and wait for callbacks
        rospy.spin()

    def axis_callback(self, msg):
        if len(msg.data) >= 2:
            # Extract x_axis_value and y_axis_value from the received Int32MultiArray
            x_axis_value = msg.data[2]
            y_axis_value = msg.data[1]

            # Map the joystick values to a range of -100 to 100
            linear_velocity = map_value(y_axis_value, 0, 100, -100, 100)
            angular_velocity = map_value(x_axis_value, 0, 100, -100, 100)

            # Control rover movement based on the mapped linear and angular velocities
            self.rover.move(linear_velocity, angular_velocity)

if __name__ == '__main__':
    try:
        # Create an instance of the JoystickController class when the script is run
        joystick_controller = JoystickController()
    finally:
        # No need to clean up WiringPi in this case
        pass
