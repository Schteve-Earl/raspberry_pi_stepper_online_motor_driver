#!/usr/bin/env python3

import rospy
import actionlib
from rpi_stepper_motor_driver.msg import ControlMotorAction
# import RPi.GPIO as GPIO


class MotorControl(object):
    def __init__(self):
        self.motor_as = actionlib.SimpleActionServer("/stepper_motor_control", ControlMotorAction, self.callback)
        self.pulses_per_rotation_ = rospy.get_param("/motor_parameters/pulses_per_rotation")

    def callback(self, goal):
        result = action.ControlMotorResult
        result.result = self.execute_move()
        self.motor_as.set_succeeded(result)

    def set_direction(self, direction: int):
        self.direction_ = direction

    def set_speed(self, speed: float):
        if speed < 0.01:
            speed = 0.01
        self.speed_ = self.convert_speed_to_ms(speed)

    def set_rotation(self, rotation: float):
        self.rotation_ = self.convert_rotation_to_pulse(rotation)

    def execute_move(self):
        if self.direction_ == 0:
            print("would be setting gpio here")
        current_pulse = 0
        while current_pulse < self.rotation_:
            current_pulse += 1
        return True

    def convert_speed_to_ms(self, speed: float):
        return 1 / (self.pulses_per_rotation_ / 6.2832 * speed)

    def convert_rotation_to_pulse(self, rotation: float):
        return self.pulses_per_rotation_ / 6.2832 * rotation

if __name__ == '__main__':
    rospy.init_node('motor_control_node')

    m = MotorControl()

    rospy.spin()