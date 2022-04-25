#!/usr/bin/env python3

import rospy
import actionlib
import action
import RPi.GPIO as GPIO


class MotorControl(object):
    def __init__(self):
        self.__as = actionlib.SimpleActionServer("/stepper_motor_control", action.ControlMotor, self.__callback())
        self.pulses_per_rotation_ = rospy.get_param("/motor_parameters/pulses_per_rotation")

    def __callback(self, goal):
        result = action.ControlMotorResult
        result.result = self.__execute_move()
        self.__as.set_succeeded(result)

    def __set_direction(self, direction: int):
        self.direction_ = direction

    def __set_speed(self, speed: float):
        if speed < 0.01:
            speed = 0.01
        self.speed_ = self.__convert_speed_to_ms(speed)

    def __set_rotation(self, rotation: float):
        self.rotation_ = self.__convert_rotation_to_pulse(rotation)

    def __execute_move(self):
        if self.direction_ == 0:

        current_pulse = 0
        while current_pulse < self.rotation_:

        return True

    def __convert_speed_to_ms(self, speed: float):
        return 1 / (self.pulses_per_rotation_ * speed)

    def __convert_rotation_to_pulse(self, rotation: float):
        return self.pulses_per_rotation_ / 6.2832 * rotation

if __name__ == '__main__':
    rospy.init_node('motor_control_node')

    m = MotorControl()

    rospy.spin()