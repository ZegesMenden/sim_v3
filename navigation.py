from physics import *
from controlMath import *
from dataManagement import *

import numpy as np
import random

class DOF6:

    def __init__(self, name):

        self.name = name

        self.time = 0.0

        self.dryMass = 0.0
        self.mass = 0.0

        self.mmoi = vector3(0.0, 0.0, 0.0)
        self.gravity = vector3(0.0, 0.0, 0.0)

        self.accelerationLocal = vector3(0.0, 0.0, 0.0)
        self.accelerationInertial = vector3(0.0, 0.0, 0.0)
        self.velocityInertial = vector3(0.0, 0.0, 0.0)
        self.positionInertial = vector3(0.0, 0.0, 0.0)

        self.rotationalVelocity = vector3(0.0, 0.0, 0.0)
        self.rotationalAcceleration = vector3(0.0, 0.0, 0.0)

        self.rotation_euler = vector3(0.0, 0.0, 0.0) # x is roll, y is pitch, z is yaw
        self.rotaiton_quaternion = Quaternion()
        
    def oriFromEuler(self, x, y, z):
        self.rotation_euler = vector3(x, y, z)
        self.rotaiton_quaternion = Quaternion().eulerToQuaternion(self.rotation_euler)
    
    def addForce(self, force):
        self.accelerationLocal += force / self.mass

    def addTorque(self, torque):
        self.rotationalAcceleration.x += torque.x / self.mmoi.x
        self.rotationalAcceleration.y += torque.y / self.mmoi.y
        self.rotationalAcceleration.z += torque.z / self.mmoi.z

    def update(self, dt):
        
        self.rotationalVelocity += (self.rotationalAcceleration) * dt
        self.rotationalVelocity.x = 0.0
        self.rotaiton_quaternion.updateOrientation(self.rotationalVelocity.x, self.rotationalVelocity.y, self.rotationalVelocity.z, dt)
        self.rotation_euler = self.rotaiton_quaternion.quaternionToEuler()

        self.accelerationInertial = self.rotaiton_quaternion.rotateVector(vector3(self.accelerationLocal.x, self.accelerationLocal.y, self.accelerationLocal.z))
        self.accelerationInertial += self.gravity

        self.velocityInertial += self.accelerationInertial * dt
        self.positionInertial += self.velocityInertial * dt

        if self.positionInertial.x <= 0:
            self.velocityInertial = vector3(0.0, 0.0, 0.0)
            self.rotationalVelocity = vector3(0.0, 0.0, 0.0)
            self.positionInertial = vector3(0.0, self.positionInertial.y, self.positionInertial.z)

        self.rotationalAcceleration = vector3(0.0, 0.0, 0.0)
        self.accelerationInertial = vector3(0.0, 0.0, 0.0)
        self.accelerationLocal = vector3(0.0, 0.0, 0.0)

        # self.dataLogger.recordVariable("time", self.time)
        # self.dataLogger.recordVariable("ori_x_roll", self.rotation_euler.x)
        # self.dataLogger.recordVariable("ori_y_pitch", self.rotation_euler.y)
        # self.dataLogger.recordVariable("ori_z_yaw", self.rotation_euler.z)
        # self.dataLogger.recordVariable("ori_x_roll_sensed", self.rotation_euler.x)
        # self.dataLogger.recordVariable("ori_y_pitch_sensed", self.rotation_euler.y)
        # self.dataLogger.recordVariable("ori_z_yaw_sensed", self.rotation_euler.z)
        # self.dataLogger.recordVariable("reaction_wheel_output_x_roll", 0.0)
        # self.dataLogger.recordVariable("actuator_output_y_pitch", self.)
        # self.dataLogger.recordVariable("actuator_output_z_yaw", self.)
        # self.dataLogger.recordVariable("ori_x_roll_setpoint", self.)
        # self.dataLogger.recordVariable("ori_y_pitch_setpoint", self.)
        # self.dataLogger.recordVariable("ori_z_yaw_setpoint", self.)
        # self.dataLogger.recordVariable("ori_x_roll_velocity", self.)
        # self.dataLogger.recordVariable("ori_y_pitch_velocity", self.)
        # self.dataLogger.recordVariable("ori_z_yaw_velocity", self.)
        # self.dataLogger.recordVariable("ori_x_roll_velocity_sensed", self.)
        # self.dataLogger.recordVariable("ori_y_pitch_velocity_sensed", self.)
        # self.dataLogger.recordVariable("ori_z_yaw_velocity_sensed", self.)
        # # self.dataLogger.recordVariable("ori_x_acceleration", self.)####", self.)
        # # self.dataLogger.recordVariable("ori_y_acceleration", self.)####", self.)
        # # self.dataLogger.recordVariable("ori_z_acceleration", self.)####", self.)
        # self.dataLogger.recordVariable("x_position", self.)
        # self.dataLogger.recordVariable("y_position", self.)
        # self.dataLogger.recordVariable("z_position", self.)
        # self.dataLogger.recordVariable("x_position_sensed", self.)
        # self.dataLogger.recordVariable("y_position_sensed", self.)
        # self.dataLogger.recordVariable("z_position_sensed", self.)
        # self.dataLogger.recordVariable("x_velocity", self.)
        # self.dataLogger.recordVariable("y_velocity", self.)
        # self.dataLogger.recordVariable("z_velocity", self.)
        # self.dataLogger.recordVariable("x_velocity_sensed", self.)
        # self.dataLogger.recordVariable("y_velocity_sensed", self.)
        # self.dataLogger.recordVariable("z_velocity_sensed", self.)
        # self.dataLogger.recordVariable("resultant_velocity", self.)
        # self.dataLogger.recordVariable("resultant_velocity_sensed", self.)
        # self.dataLogger.recordVariable("x_acceleration", self.)
        # self.dataLogger.recordVariable("y_acceleration", self.)
        # self.dataLogger.recordVariable("z_acceleration", self.)
        # self.dataLogger.recordVariable("x_acceleration_sensed", self.)
        # self.dataLogger.recordVariable("y_acceleration_sensed", self.)
        # self.dataLogger.recordVariable("z_acceleration_sensed", self.)
        # self.dataLogger.recordVariable("x_acceleration_inertial", self.)
        # self.dataLogger.recordVariable("y_acceleration_inertial", self.)
        # self.dataLogger.recordVariable("z_acceleration_inertial", self.)
        # self.dataLogger.recordVariable("x_acceleration__inertial_sensed", self.)
        # self.dataLogger.recordVariable("y_acceleration__inertial_sensed", self.)
        # self.dataLogger.recordVariable("z_acceleration__inertial_sensed", self.)
        # self.dataLogger.recordVariable("resultant_acceleration", self.)
        # self.dataLogger.recordVariable("resultant_acceleration_sensed", self.)
        # self.dataLogger.recordVariable("thrust", self.)
        # self.dataLogger.recordVariable("mass", self.)
        
