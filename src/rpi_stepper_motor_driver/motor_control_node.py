#!/usr/bin/env python3

import rospy
import actionlib
import rpi_stepper_motor_driver.msg



class MotorControl(object):
    def __init__(self):
        self.motor_as = actionlib.SimpleActionServer("stepper_motor_control", rpi_stepper_motor_driver.msg.ControlMotorAction, self.callback)

        self.load_parameters()

        # GPIO only works on RPi, so only load and configure if on a raspberry pi
        if not self.sim_:
            import RPi.GPIO as GPIO
            self.set_gpio_pins()

    def load_parameters(self):
        # Driver Parameters
        self.sim_ = rospy.get_param("/rpi_stepper_motor_driver/driver_parameters/sim")
        self.minimum_speed_ = rospy.get_param("/rpi_stepper_motor_driver/driver_parameters/minimum_speed")

        # Motor parameters
        self.pulses_per_rotation_ = rospy.get_param("/rpi_stepper_motor_driver/motor_parameters/pulses_per_rotation")
        self.step_pin = rospy.get_param("/rpi_stepper_motor_driver/controller_paramters/step_pin")
        self.direction_pin = rospy.get_param("/rpi_stepper_motor_driver/controller_paramters/direction_pin")
        
        
    def set_gpio_pins(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.direction_pin, GPIO.OUT)

    def callback(self, goal):
        result = rpi_stepper_motor_driver.msg.ControlMotorResult()
        if not self.set_direction(goal.direction) or not self.set_speed(goal.speed) or not self.set_rotation(goal.rotation):
            rospy.logerr("Aborting move")
            result.result = False
            self.motor_as.set_succeeded(result)
            return
        
        result.result = self.execute_move()
        self.motor_as.set_succeeded(result)

    def set_direction(self, direction: int):
        self.direction = direction
        if not self.sim_:
            GPIO.output(self.direction_pin, self.direction)
        return True

    def set_speed(self, speed: float):
        if speed < self.minimum_speed_:
            rospy.logerr("Speed must be more than 0.01 rad/s")
            return False
        self.dwell_time = self.convert_speed_to_ms(speed)
        return True

    def set_rotation(self, rotation: float):
        if rotation <= 0:
            rospy.logerr("Rotation must be greater than 0 radinas")
            return False
        self.rotation_ = self.convert_rotation_to_pulse(rotation)
        return True

    def execute_move(self):
        current_pulse = 0
        while current_pulse < self.rotation_:
            if not self.sim_:
                GPIO.output(self.step_pin, GPIO.HIGH)
                GPIO.output(self.step_pin, GPIO.LOW)
            rospy.sleep(self.dwell_time)
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