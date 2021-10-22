import numpy as np
import random
from physics import *

def calculateAngleFromDesiredTorque(moment_arm, force, mmoi, desired_torque):
    calcval = desired_torque * mmoi / force / moment_arm
    return np.arcsin(calcval)

def positive_or_negative():
    if random.random() < 0.5:
        return 1
    else:
        return -1

class PID:
    def __init__(self, kP, kI, kD, setpoint, iMax, usePONM):
        
        #assigning gains to user inputs
        self.kP = kP 
        self.kI = kI
        self.kD = kD

        #the value that the PID controller wants the input to become
        self.setPoint = setpoint

        # you might not see this in your run-of-the-mill PID library, this is just a limit on how bit the integral can get
        self.iMax = iMax 

        #you are even less likely to see this in your run-of-the-mill PID library
        #this is basically a different implementation of the proportional gain, where instead of looking at the error
        #the proportional part of the controller looks at the change between the initial sensed value and the current sensed value
        
        self.usePONM = usePONM
        
        self.proportional = 0

        #this variable stores the most recent process variable, so that when update is called the PID controller can
        #know not only the current error, but also the speed at which it is approaching or departing from the setpoint!
        #this helps with reducing overshoot by feeding it into the derivitive value, which basically functions as a break on a car
        self.lastProcess = 0

        #this is the integral, which helps combat steady state error. Eg. your rocket is stable, and has canceled all rotation on
        #it's body, but its 10 degrees off target! the integral is crucial here as it increases over time with the error
        self.integral = 0
        self.error = 0

        self.derivitive = 0 
        #for dataloging purpouses

        self.output = 0

    def setSetpoint(self, setpoint):
        self.setPoint = setpoint

    def zeroIntegral(self):
        self.integral = 0

    def compute(self, process, dt):
        
        change = process - self.lastProcess

        self.lastProcess = process

        self.error = self.setPoint - process

        if self.usePONM == True:
            #whats happening here is the proportional changing with the process variable
            self.proportional -= change * self.kP
        else:
            self.proportional = self.error * self.kP

        #checking that the integral will not exceed the maximum value allowed by the user
        if abs(self.integral + self.error * self.kI * dt) < self.iMax:
            self.integral += self.error * self.kI * dt

        # if abs(self.error > 5):
        #     if change < 0:
        #         self.derivitive = ((change / dt) + 30) * self.kD
        #         if self.derivitive < 0:
        #             self.derivitive = 0
        #     if change > 0:
        #         self.derivitive = ((change / dt) - 30) * self.kD
        #         if self.derivitive > 0:
        #             self.derivitive = 0
        # else:
    
        self.derivitive = change / dt * self.kD

        if self.usePONM == True:
            self.output = self.proportional + self.integral - self.derivitive
        else:
            self.output = self.proportional + self.integral - self.derivitive

class FSF:
    #gains
    # fsf_gains = np.matrix([0,0,0,0])

    #setpoints (ori, rotVel, position, velocity)
    # fsf_setpoint = np.array([0,0,0,0])

    lastOri = 0
    lastPos = 0

    I = 0
    output = 0

    def __init__(self, _gains, _setpoint, iGain):
        self.fsf_gains = _gains
        self.fsf_setpoint = _setpoint
        self.iGain = iGain

    def compute(self, pos, ori, dt):
        x = np.matrix([
            [ori - self.fsf_setpoint[0]],
            [((ori - self.lastOri) / dt) - self.fsf_setpoint[1]],
            [pos - self.fsf_setpoint[2]],
            [((pos - self.lastPos)) - self.fsf_setpoint[3]]
        ])
        out = -self.fsf_gains * x
        self.output = np.sum(out)
        self.I += self.output * self.iGain
        self.lastOri = ori
        self.lastPos = pos
        self.output += self.I
    
    def setSetpoint(self, ori, oriRate, pos, vel):
        self.fsf_gains = np.array([ori, oriRate, pos, vel])
    
    def reset(self):
        self.output = 0
        self.lastPos = 0
        self.lastOri = 0
        self.I = 0

class Sensor:

    def __init__(self):

        self.accelerometerNoise = vector3(0.0, 0.0, 0.0)
        self.accelerometerOffset = vector3(0.0, 0.0, 0.0)

        self.gyroscopeNoise = vector3(0.0, 0.0, 0.0)
        self.gyroscopeBias = vector3(0.0, 0.0, 0.0)

        self.oriRates = vector3(0.0, 0.0, 0.0)
        self.orientation_quat = Quaternion()
        self.orientation_euler = vector3(0.0, 0.0, 0.0)

        self.accelerationLocal = vector3(0.0, 0.0, 0.0)
        self.accelerationInertial = vector3(0.0, 0.0, 0.0)
        self.velocityInertial = vector3(0.0, 0.0, 0.0)
        self.positionInertial = vector3(0.0, 0.0, 0.0)

    def update(self, acceleration, oriRate, gravity, dt):
        
        self.oriRates.x = oriRate.x + self.gyroscopeBias.x + ((random.randint(60, 100) / 100) * self.gyroscopeNoise.x * positive_or_negative())
        self.oriRates.y = oriRate.y + self.gyroscopeBias.y + ((random.randint(60, 100) / 100) * self.gyroscopeNoise.y * positive_or_negative())
        self.oriRates.z = oriRate.z + self.gyroscopeBias.z + ((random.randint(60, 100) / 100) * self.gyroscopeNoise.z * positive_or_negative())
        
        self.orientation_quat.updateOrientation( self.oriRates.x, self.oriRates.y, self.oriRates.z, dt )
        self.orientation_euler = self.orientation_quat.quaternionToEuler()

        self.accelerationLocal.x = acceleration.x + ((random.randint(60, 100) / 100) * self.accelerometerNoise.x * positive_or_negative())
        self.accelerationLocal.y = acceleration.y + ((random.randint(60, 100) / 100) * self.accelerometerNoise.y * positive_or_negative())
        self.accelerationLocal.z = acceleration.z + ((random.randint(60, 100) / 100) * self.accelerometerNoise.z * positive_or_negative())

        self.accelerationInertial = self.orientation_quat.rotateVector(self.accelerationLocal)
        self.accelerationInertial += gravity

        self.velocityInertial += self.accelerationInertial * dt
        self.positionInertial += self.velocityInertial * dt
