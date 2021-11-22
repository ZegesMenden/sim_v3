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

def LPF(new, old, gain):
    return new*(1-gain) + old*gain

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

class FSF_oriPos:
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

class FSF_ori:
    lastOri = 0
    I = 0
    output = 0

    def __init__(self, _gains, _setpoint, iGain):
        self.fsf_gains = _gains
        self.fsf_setpoint = _setpoint
        self.iGain = iGain

    def compute(self, ori, dt):
        x = np.matrix([
            [ori - self.fsf_setpoint[0]],
            [((ori - self.lastOri) / dt) - self.fsf_setpoint[1]]])
        out = -self.fsf_gains * x
        self.output = np.sum(out)
        self.I += self.output * self.iGain
        self.lastOri = ori
        self.output += self.I
    
    def setSetpoint(self, ori, oriRate):
        self.fsf_gains = np.array([ori, oriRate])
    
    def reset(self):
        self.output = 0
        self.lastOri = 0
        self.I = 0

class FSF_ori_test:

    def __init__(self, gain_a, gain_b):
        self.gain_a = gain_a
        self.gain_b = gain_b
        self.setpoint = 0.0
        self.out = 0.0
        self.lastOri = 0.0

    def compute(self, ori, oriRate):
        self.output = -self.gain_a * (ori - self.setpoint)
        self.output += -self.gain_b * oriRate
        self.lastOri = ori
    def getOutput(self):
        return self.output

class NAVController:

    def __init__(self):

        self.accelBias = vector3(0.0, 0.0, 0.0)

        self.gyroscopeBias = vector3(0.0, 0.0, 0.0)

        self.barometerAlt = 0.0
        self.barometerVel = 0.0
        self.barometerTime = 0.0

        self.oriRates = vector3(0.0, 0.0, 0.0)
        self.oriRatesFiltered = vector3(0.0, 0.0, 0.0)
        self.lastOriRates = vector3(0.0, 0.0, 0.0)

        self.orientation_quat = Quaternion()
        self.orientation_euler = vector3(0.0, 0.0, 0.0)

        self.accelerationLocal = vector3(0.0, 0.0, 0.0)
        self.accelerationLocalFiltered = vector3(0.0, 0.0, 0.0)
        self.lastAccelerationLocal = vector3(0.0, 0.0, 0.0)

        self.accelerationInertial = vector3(0.0, 0.0, 0.0)
        
        self.velocityInertial = vector3(0.0, 0.0, 0.0)
        self.positionInertial = vector3(0.0, 0.0, 0.0)

        self.debiasCount = 0

        self.debiased = False
        self.inFlight = False

    def update(self, acceleration, rotationalVel, gravity, dt, time):

        self.accelerationLocal = acceleration
        self.oriRates = rotationalVel

        # if self.inFlight == True and self.debiased == True:
        self.accelerationLocalFiltered = self.accelerationLocal - self.accelBias
        # self.accelerationLocalFiltered = LPF(self.accelerationLocalFiltered, self.lastAccelerationLocal, 0.05)

        self.oriRatesFiltered = self.oriRates# - self.gyroscopeBias
        
        # self.oriRatesFiltered = LPF(self.oriRatesFiltered, self.lastOriRates, 0.05)
        
        self.orientation_quat.updateOrientation( self.oriRatesFiltered.x, self.oriRatesFiltered.y, self.oriRatesFiltered.z, dt )
        self.orientation_euler = self.orientation_quat.quaternionToEuler()

        self.accelerationInertial = self.orientation_quat.rotateVector(self.accelerationLocalFiltered)
        self.accelerationInertial += gravity

        self.velocityInertial += self.accelerationInertial * dt
        self.positionInertial += self.velocityInertial * dt

        self.velocityInertial.x = LPF(self.velocityInertial.x, self.barometerVel, 0.05 * (((time - self.barometerTime) / 40) + 1))
        self.positionInertial.x = LPF(self.positionInertial.x, self.barometerAlt, 0.05 * (((time - self.barometerTime) / 40) + 1))

        if self.positionInertial.x <= 0:
            self.positionInertial.x = 0
            self.velocityInertial.x = 0
        # else:
        #     self.accelerationLocalFiltered = self.accelerationLocal
        #     self.oriRatesFiltered = self.oriRates
    
    def accelOri(self, accel):
        self.orientation_euler.y = np.arctan2(accel.y, accel.z)
        self.orientation_euler.z = np.arctan2(accel.z, accel.y)
        self.orientation_quat = self.orientation_quat.eulerToQuaternion(self.orientation_euler.x, self.orientation_euler.y, self.orientation_euler.z)

    def passBarometerData(self, barometerAlt, barometerVel, time):
        self.barometerAlt = barometerAlt
        self.barometerVel = barometerVel
        self.barometerTime = time
    
    def measureDebias(self, acceleration, rotationalVel):
        if self.inFlight == False and self.debiased == False:
            self.debiasCount += 1
            self.accelBias += acceleration
            self.gyroscopeBias += rotationalVel

    def debias(self):
        if self.debiased == False:
            self.accelBias /= self.debiasCount
            self.gyroscopeBias /= self.debiasCount
            self.debiased = True

class barometer:

    def __init__(self):
        self.altitude = 0.0
        self.noise = 0.0
        self.lastRead = 0.0
        self.lastAltitude = 0.0
        self.lastVelocity = 0.0
        self.velocity = 0.0
        self.readDelay = 0.0
        self.pressureEventOffset = 0.0
        self.timeSincePressureEvent = 0.0
    
    def read(self, altitude, time):
        if time > self.lastRead + self.readDelay:
            if self.pressureEventOffset > 0:
                self.pressureEventOffset -= self.pressureEventOffset * 0.5 * (time - self.lastRead)
            self.altitude = altitude + ((random.randint(0, 100) / 100) * self.noise * positive_or_negative())
            self.velocity = (self.altitude - self.lastAltitude) / (time - self.lastRead)
            self.lastRead = time
            self.lastAltitude = self.altitude
            self.lastVelocity = self.velocity
    
    def pressureEvent(self, effect, time):
        self.pressureEventOffset = effect
        self.timeSincePressureEvent = time

class IMU6DOF:

    def __init__(self):
        
        self.accel = vector3(0.0, 0.0, 0.0)
        self.oriRates = vector3(0.0, 0.0, 0.0)
        
        self.gyroBias = vector3(0.0, 0.0, 0.0)
        self.accelBias = vector3(0.0, 0.0, 0.0)
        
        self.accelScale = vector3(0.0, 0.0, 0.0)
        self.gyroScale = vector3(0.0, 0.0, 0.0)
        
        self.gyroNoise = vector3(0.0, 0.0, 0.0)
        self.accelNoise = vector3(0.0, 0.0, 0.0)
        
        self.sampleRateAccel = 0.0
        self.sampleRateGyro = 0.0

        self.lastReadAccel = 0.0
        self.lastReadGyro = 0.0

    def readAccel(self, trueAccelerations, time):

        if time > self.lastReadAccel + self.sampleRateAccel:
            
            self.lastReadAccel = time

            self.accel.x = np.random.normal(trueAccelerations.x, self.accelNoise.x, 1)[0]
            self.accel.y = np.random.normal(trueAccelerations.y, self.accelNoise.y, 1)[0]
            self.accel.z = np.random.normal(trueAccelerations.z, self.accelNoise.z, 1)[0]

            self.accel += self.accelBias

            if self.accel.x > self.accelScale.x:
                self.accel.x = self.accelScale.x
            if self.accel.x < -self.accelScale.x:
                self.accel.x = -self.accelScale.x
            if self.accel.y > self.accelScale.y:
                self.accel.y = self.accelScale.y
            if self.accel.y < -self.accelScale.y:
                self.accel.y = -self.accelScale.y
            if self.accel.z > self.accelScale.z:
                self.accel.z = self.accelScale.z
            if self.accel.z < -self.accelScale.z:
                self.accel.z = -self.accelScale.z

    def readGyro(self, trueOriRates, time):

        if time > self.lastReadGyro + self.sampleRateGyro:

            self.lastReadGyro = time

            self.oriRates.x = trueOriRates.x + np.random.normal(0, self.gyroNoise.x, 1)[0]
            self.oriRates.y = trueOriRates.y + np.random.normal(0, self.gyroNoise.y, 1)[0]
            self.oriRates.z = trueOriRates.z + np.random.normal(0, self.gyroNoise.z, 1)[0]

            self.oriRates += self.gyroBias

            if self.oriRates.x > self.gyroScale.x:
                self.oriRates.x = self.gyroScale.x
            if self.oriRates.x < -self.gyroScale.x:
                self.oriRates.x = -self.gyroScale.x

            if self.oriRates.y > self.gyroScale.y:
                self.oriRates.y = self.gyroScale.y
            if self.oriRates.y < -self.gyroScale.y:
                self.oriRates.y = -self.gyroScale.y

            if self.oriRates.z > self.gyroScale.z:
                self.oriRates.z = self.gyroScale.z
            if self.oriRates.z < -self.gyroScale.z:
                self.oriRates.z = -self.gyroScale.z
        