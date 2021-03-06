import math
from typing import AsyncContextManager
import numpy as np
import random
from physics import *

def calculateAngleFromDesiredTorque(moment_arm, force, mmoi, desired_torque):
    calcval = desired_torque * mmoi / force / moment_arm
    return math.asin(calcval)

def positive_or_negative():
    if random.random() < 0.5:
        return 1
    else:
        return -1

def LPF(new, old, gain):
    return new*(1-gain) + old*gain

class PID:
    def __init__(self, kP, kI, kD, setpoint, iMax, usePONM):
        
        #assigning gains to user imathuts
        self.kP = kP 
        self.kI = kI
        self.kD = kD

        #the value that the PID controller wants the imathut to become
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

        self.derivitive = change / dt * self.kD

        if self.usePONM == True:
            self.output = self.proportional + self.integral - self.derivitive
        else:
            self.output = self.proportional + self.integral - self.derivitive

class FSF:

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

class kalman1dof:

    # just a python implementation of atlas aerospace's arduino kalman filter:
    # https://github.com/atlas-aerospace-yt/Arduino-Kalman-Filter

    def __init__(self) -> None:

        self.a = vector3(1, 1, 1)
        self.b = vector3(1, 1, 1)
        self.c = vector3(1, 1, 1)

        self.q = vector3()
        self.r = vector3()
        self.u = vector3()

        self.x_hat = vector3()
        self.p = vector3()

    def update(self, sensor_reading) -> None:

        x_hat_minus_1 = self.a * self.x_hat + self.b * self.u

        self.p = self.a * self.p * self.a + self.q

        k = self.p * self.c * ( vector3(1, 1, 1) / ( self.c * self.p * self.c * self.r ))
        x_hat_minus_1 += k * (sensor_reading - self.c * x_hat_minus_1)

        self.p = (vector3(1, 1, 1) - k * self.c) * self.p

        self.x_hat = x_hat_minus_1

    def output(self) -> float:
        return self.x_hat

    def set_gains(self, q, r) -> None:

        self.q = q
        self.r = r

class kalmanPosition:

    def __init__(self) -> None:

        self.P = [0.0, 0.0, 0.0, 0.0]
        self.vel = 0.0
        self.pos = 0.0

        self.Q = 0.0
        self.R = 0.0

    def update_accel(self, accel, dt) -> None:

        self.vel += accel * dt
        self.pos += self.vel * dt
        q_dtdt = self.Q * (dt * dt)
        self.P[0] += ( self.P[2] + self.P[1] + ( self.P[3] + q_dtdt * 0.25 ) * dt ) * dt
        self.P[1] += ( self.P[3] + q_dtdt * 0.5 ) * dt
        self.P[2] = self.P[1]
        self.P[3] += q_dtdt

    def update_position(self, pos) -> None:

        y = self.pos - pos

        inv = 1.0 / ( self.P[0] + self.R )

        K_a = self.P[0] * inv
        K_b = self.P[2] * inv

        self.pos += K_a * y
        self.vel += K_b * y

        self.vel += y
        self.vel /= 2.0

        self.pos += pos
        self.pos /= 2.0

        self.P[0] -= K_a * self.P[0]
        self.P[1] -= K_a * self.P[1]
        self.P[2] -= K_b * self.P[0]
        self.P[3] -= K_b * self.P[1]

    def get_position(self) -> float:

        return self.pos

    def get_velocity(self) -> float:

        return self.vel

class NAVController:

    def __init__(self) -> None:

        self.accelBias = vector3(0.0, 0.0, 0.0)

        self.gyroscopeBias = vector3(0.0, 0.0, 0.0)

        self.barometerAlt = 0.0
        self.barometerVel = 0.0
        self.barometerTime = 0.0

        self.oriRates = vector3(0.0, 0.0, 0.0)
        self.lastOriRates = vector3(0.0, 0.0, 0.0)

        self.orientation_quat = Quaternion()
        self.orientation_euler = vector3(0.0, 0.0, 0.0)

        self.accelerationLocal = vector3(0.0, 0.0, 0.0)
        self.lastAccelerationLocal = vector3(0.0, 0.0, 0.0)

        self.accelerationInertial = vector3(0.0, 0.0, 0.0)
        
        self.velocityInertial = vector3(0.0, 0.0, 0.0)
        self.positionInertial = vector3(0.0, 0.0, 0.0)

        self.debiasCount = 0

        self.debiased = False
        self.inFlight = False

        self.posKF_x = kalmanPosition()
        self.posKF_y = kalmanPosition()
        self.posKF_z = kalmanPosition()

    def update(self, acceleration, rotationalVel, gravity, dt) -> None:

        self.accelerationLocal = acceleration - self.accelBias
        self.oriRates = rotationalVel - self.gyroscopeBias
        
        if self.inFlight == False:
            self.positionInertial = vector3(0, 0 ,0)
            self.velocityInertial = vector3(0, 0, 0)
        
        ang = self.oriRates.norm()

        self.orientation_quat *= Quaternion(0, 0, 0, 0).fromAxisAngle(ang*dt, self.oriRates.x/ang, self.oriRates.y/ang, self.oriRates.z/ang)

        self.orientation_euler = self.orientation_quat.quaternionToEuler()
        
        self.accelerationInertial = self.orientation_quat.rotateVector(self.accelerationLocal)

        self.accelerationInertial += gravity

        self.posKF_x.update_accel(self.accelerationInertial.x, dt)
        self.posKF_y.update_accel(self.accelerationInertial.y, dt)
        self.posKF_z.update_accel(self.accelerationInertial.z, dt)

        self.velocityInertial.x = self.posKF_x.vel
        self.velocityInertial.y = self.posKF_y.vel
        self.velocityInertial.z = self.posKF_z.vel

        self.positionInertial.x = self.posKF_x.pos
        self.positionInertial.y = self.posKF_y.pos
        self.positionInertial.z = self.posKF_z.pos

        if self.accelerationLocal.x > 10:
            self.inFlight = True

        if self.positionInertial.x <= 0:
            self.positionInertial.x = 0
            self.velocityInertial.x = 0
        # else:
        #     self.accelerationLocalFiltered = self.accelerationLocal
        #     self.oriRatesFiltered = self.oriRates
    
    def accelOri(self, accel) -> None:
        q = Quaternion().fromVector(self.orientation_quat.rotateVector(accel)) * Quaternion(0, 1, 0, 0)
        q.w = 1 - q.w
        self.orientation_quat = Quaternion().fromVector(self.orientation_quat.conj().rotateVector(q.fractional(0.5)))

    def passBarometerData(self, barometerAlt, barometerVel, time) -> None:
        self.barometerAlt = barometerAlt
        self.barometerVel = barometerVel
        self.barometerTime = time
        self.posKF_x.update_position(barometerAlt)

    def passGPSData(self, gpsData) -> None:
        
        if isinstance(gpsData, vector3):
            self.posKF_x.update_position(gpsData.x)
            self.posKF_y.update_position(gpsData.y)
            self.posKF_z.update_position(gpsData.z)
    
    def measureDebias(self, acceleration, rotationalVel) -> None:
        if self.inFlight == False and self.debiased == False:
            self.debiasCount += 1
            self.accelBias += acceleration
            self.gyroscopeBias += rotationalVel

    def debias(self) -> None:
        if self.debiased == False and self.debiasCount > 0:
            self.accelBias /= self.debiasCount
            self.gyroscopeBias /= self.debiasCount
            self.debiased = True

class barometer:

    def __init__(self) -> None:
        self.altitude = 0.0
        self.noise = 0.0
        self.lastRead = 0.0
        self.lastAltitude = 0.0
        self.lastVelocity = 0.0
        self.velocity = 0.0
        self.readDelay = 0.0
        self.pressureEventOffset = 0.0
        self.timeSincePressureEvent = 0.0
    
    def read(self, altitude, time) -> None:
        if time > self.lastRead + self.readDelay:
            if self.pressureEventOffset > 0:
                self.pressureEventOffset -= self.pressureEventOffset * 0.5 * (time - self.lastRead)
            self.altitude = altitude + ((random.randint(0, 100) / 100) * self.noise * positive_or_negative())
            self.velocity = (self.altitude - self.lastAltitude) / (time - self.lastRead)
            self.lastRead = time
            self.lastAltitude = self.altitude
            self.lastVelocity = self.velocity
    
    def pressureEvent(self, effect, time) -> None:
        self.pressureEventOffset = effect
        self.timeSincePressureEvent = time

class IMU6DOF:

    def __init__(self) -> None:
        
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

    def readAccel(self, trueAccelerations, time) -> None:

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

    def readGyro(self, trueOriRates, time) -> None:

        if time > self.lastReadGyro + self.sampleRateGyro:

            self.lastReadGyro = time

            self.oriRates.x = trueOriRates.x + np.random.normal(0, self.gyroNoise.x, 1)[0] * DEG_TO_RAD
            self.oriRates.y = trueOriRates.y + np.random.normal(0, self.gyroNoise.y, 1)[0] * DEG_TO_RAD
            self.oriRates.z = trueOriRates.z + np.random.normal(0, self.gyroNoise.z, 1)[0] * DEG_TO_RAD

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

class GPS:

    def __init__(self) -> None:
        
        self.measuredPosition = vector3()
        self.measuredVelocity = vector3()
        self.signs = vector3(random.choice([-1, 1]), random.choice([-1, 1]), random.choice([-1, 1]))
        self.lastRead = 0.0
        self.accuracy = 0.0

    def update(self, position, velocity, dt, time) -> None:
        
        self.accuracy += random.randint(0, 100) / 1000.0 * random.choice([-1, 1])

        if self.accuracy < 0.1:
            self.accuracy = 0.1

        if self.accuracy > 0.75:
            self.accuracy = 0.75

        self.measuredPosition = position + (vector3(self.accuracy, self.accuracy, self.accuracy) * self.signs)
        self.measuredVelocity = velocity + (vector3(self.accuracy, self.accuracy, self.accuracy) * self.signs * dt)

        self.lastRead = time