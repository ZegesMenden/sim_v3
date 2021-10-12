import numpy as np
import random
from math import isclose

from pygame import error

DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

def clamp(n, minn, maxn): # Clamps output to a range
    return max(min(maxn, n), minn)
#thanks orlando

def rotate(x, y, theta): # Rotates a 2D vector by an angle theta, returns a Vector3 with XY and a 0 Z component
    cs = np.cos(theta)
    sn = np.sin(theta)

    rotated_x = x * cs - y * sn
    rotated_y = x * sn + y * cs

    return vector3(rotated_x, rotated_y, 0)
#thanks orlando again

class vector3:

    def __init__(self, x, y, z):

        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, vector):

        if isinstance(vector, vector3):

            x = self.x * vector.x
            y = self.y * vector.y
            z = self.y * vector.z

        else:

            x = self.x * vector
            y = self.y * vector
            z = self.z * vector

        return vector3(x, y, z)
    
    def __truediv__(self, vector):
        
        if isinstance(vector, vector3):

            x = self.x / vector.x
            y = self.y / vector.y
            z = self.y / vector.z

        else:

            x = self.x / vector
            y = self.y / vector
            z = self.z / vector

        return vector3(x, y, z)

    def __add__(self, vector):

        x = self.x + vector.x
        y = self.y + vector.y
        z = self.z + vector.z
        
        return vector3(x, y, z)

    def __sub__(self, vector):

        x = self.x - vector.x
        y = self.y - vector.y
        z = self.z - vector.z
        
        return vector3(x, y, z)

    def __eq__(self, vector):
        
        if isinstance(vector, vector3):
            
            if isclose(self.x, vector.x, rel_tol=0.01) and isclose(self.y, vector.y, rel_tol=0.01) and isclose(self.z, vector.z, rel_tol=0.01):
                return True

            else: 
                return False

        else:
            return False

    def norm(self):

        norm = np.sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )
        
        self.x /= norm
        self.y /= norm
        self.z /= norm

        return self
    
    def len(self):

        return np.sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )

    def degRad(self):

        self.x *= DEG_TO_RAD
        self.y *= DEG_TO_RAD
        self.z *= DEG_TO_RAD

        return self

    def radDeg(self):

        self.x *= RAD_TO_DEG
        self.y *= RAD_TO_DEG
        self.z *= RAD_TO_DEG

        return self

    def __str__(self):
        return str(self.x) + ',' + str(self.y) + ',' + str(self.z)
    
class Quaternion:
    
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):

        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, quaternion):

        self.w += quaternion.w
        self.x += quaternion.x
        self.y += quaternion.y
        self.z += quaternion.z
        
        return self

    def __sub__(self, quaternion):

        self.w -= quaternion.w
        self.x -= quaternion.x
        self.y -= quaternion.y
        self.z -= quaternion.z
        
        return self

    def __mul__(self, quaternion):
        
        qNew = Quaternion(1.0, 0.0, 0.0, 0.0)

        qNew.w = ( self.w * quaternion.w ) - ( self.x * quaternion.x ) - ( self.y * quaternion.y ) - ( self.z * quaternion.z ) # im no betting man but if i were 
        qNew.x = ( self.w * quaternion.x ) + ( self.x * quaternion.w ) + ( self.y * quaternion.z ) - ( self.z * quaternion.y ) # i would bet that at least one 
        qNew.y = ( self.w * quaternion.y ) - ( self.x * quaternion.z ) + ( self.y * quaternion.w ) + ( self.z * quaternion.x ) # of the operations in this function
        qNew.z = ( self.w * quaternion.z ) + ( self.x * quaternion.y ) - ( self.y * quaternion.x ) + ( self.z * quaternion.w ) # is wrong

        # future ZegesMenden here - i was right

        return qNew

    def norm(self):

        n = np.sqrt( self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2 )

        self.w /= n
        self.x /= n
        self.y /= n 
        self.z /= n

        return self

    def conj(self):

        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def __str__(self):
        return str(self.w) + ',' + str(self.x) + ',' + str(self.y) + ',' + str(self.z)

    def __eq__(self, quaternion):
        
        if isinstance(quaternion, Quaternion):

            if isclose(self.w, quaternion.w, rel_tol=0.01) and isclose(self.x, quaternion.x, rel_tol=0.01) and isclose(self.y, quaternion.y, rel_tol=0.01) and isclose(self.z, quaternion.z, rel_tol=0.01):
                return True

            else:
                return False
    
    def eulerToQuaternion(self, roll, pitch, yaw):
        
        cr = np.cos( roll / 2 )
        cp = np.cos( pitch / 2 )
        cy = np.cos( yaw / 2 )

        sr = np.sin( roll / 2 )
        sp = np.sin( pitch / 2 )
        sy = np.sin( yaw / 2 )

        self.w = cr * cp * cy + sr * sp * sy
        self.x = sr * cp * cy - cr * sp * sy
        self.y = cr * sp * cy + sr * cp * sy
        self.z = cr * cp * sy - sr * sp * cy

        return self

    def quaternionToEuler(self):

        r = np.arctan2( 2.0 * ( self.w * self.x + self.y * self.z ), 1.0 - 2.0 * ( self.x ** 2 + self.y ** 2 ) )
        p = 2.0 * ( self.w * self.y - self.z * self.x )
        y = np.arctan2( 2.0 * ( self.w * self.z + self.x * self.y ), 1.0 - 2.0 * ( self.y ** 2 + self.z ** 2 ) )

        return vector3(r, p, y)

    def fromAxisAngle(self, t, x, y, z):
        
        sn = np.sin(t / 2.0)

        self.w = np.cos(t / 2.0)
        self.x = x * sn
        self.y = y * sn
        self.z = z * sn

        return self

    def updateOrientation(self, x, y, z, dt):

        angle = vector3(x, y, z).len()

        if angle == 0:
            angle = 1e-5
        
        qD = Quaternion(0.0, 0.0, 0.0, 0.0).fromAxisAngle(angle * dt, x/angle, y/angle, z/angle)

        qM = self * qD
        self.w = qM.w
        self.x = qM.x
        self.y = qM.y
        self.z = qM.z

        return self

    def rotateVector(self, vector):

        if isinstance(vector, vector3):

            rVector = Quaternion(0.0, vector.x, vector.y, vector.z)
            rVector = self * rVector * self.conj()

            return vector3(rVector.x, rVector.y, rVector.z)

        else:
            return False

class TVC:
    
    def __init__(self):
        
        self.commandY = 0.0
        self.commandZ = 0.0

        self.positionX = 0.0
        self.positionY = 0.0
        self.positionZ = 0.0

        self.minY = 0.0
        self.maxY = 0.0

        self.minZ = 0.0
        self.maxZ = 0.0

        self.offsetY = 0.0
        self.offsetZ = 0.0

        self.noiseY = 0.0
        self.noiseZ = 0.0

        self.servoSpeed = 0.0

        self.linkageRatioY = 0.0
        self.linkageRatioZ = 0.0

        self.lever = 0.0

        self.torque = vector3(0.0, 0.0, 0.0)
        self.acceleration = vector3(0.0, 0.0, 0.0)

    def actuate(self, command_angles, dt):

        self.commandY = command_angles.y
        self.commandZ = command_angles.z

        errorY = self.commandY - self.positionY
        errorZ = self.commandZ - self.positionZ

        speedY = self.servoSpeed * dt / self.linkageRatioY
        speedZ = self.servoSpeed * dt / self.linkageRatioZ

        errorY = clamp(errorY, -speedY, speedY)
        errorZ = clamp(errorZ, -speedZ, speedZ)
        
        self.positionX = 0.0
        self.positionY += (errorY + (random.randint(-100, 100) / 100) * self.noiseY) * DEG_TO_RAD
        self.positionZ += (errorZ + (random.randint(-100, 100) / 100) * self.noiseZ) * DEG_TO_RAD
        
        self.positionY = clamp(self.positionY, self.minY * DEG_TO_RAD, self.maxY * DEG_TO_RAD)
        self.positionZ = clamp(self.positionZ, self.minZ * DEG_TO_RAD, self.maxZ * DEG_TO_RAD)

    def calculateForces(self, thrust):
        self.acceleration.y = np.sin(self.positionY) * thrust
        self.acceleration.z = np.sin(self.positionZ) * thrust
        self.acceleration.x = thrust - self.acceleration.y - self.acceleration.z

        self.torque.y = thrust * np.sin(self.positionY) * self.lever
        self.torque.z = thrust * np.sin(self.positionZ) * self.lever