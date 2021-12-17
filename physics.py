import numpy as np
import random
from math import isclose

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

    def __init__(self, x=0.0, y=0.0, z=0.0):

        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, vector):

        if isinstance(vector, vector3):

            x = self.x * vector.x
            y = self.y * vector.y
            z = self.z * vector.z

            return vector3(x, y, z)

        else:

            x = self.x * vector
            y = self.y * vector
            z = self.z * vector

            return vector3(x, y, z)
    
    def __truediv__(self, vector):
        
        if isinstance(vector, vector3):

            x = self.x / vector.x
            y = self.y / vector.y
            z = self.z / vector.z

            return vector3(x, y, z)

        else:

            x = self.x / vector
            y = self.y / vector
            z = self.z / vector

            return vector3(x, y, z)

    def __add__(self, vector):

        if isinstance(vector, vector3):

            x = self.x + vector.x
            y = self.y + vector.y
            z = self.z + vector.z
            
            return vector3(x, y, z)
        
        else:

            x = self.x + vector
            y = self.y + vector
            z = self.z + vector

            return vector3(x, y, z)
            

    def __sub__(self, vector):

        if isinstance(vector, vector3):

            x = self.x - vector.x
            y = self.y - vector.y
            z = self.z - vector.z
            
            return vector3(x, y, z)
        
        else:

            x = self.x - vector
            y = self.y - vector
            z = self.z - vector

            return vector3(x, y, z)
            

    def __eq__(self, vector):
        
        if isinstance(vector, vector3):
            
            if isclose(self.x, vector.x, rel_tol=0.001) and isclose(self.y, vector.y, rel_tol=0.01) and isclose(self.z, vector.z, rel_tol=0.001):
                return True

            else: 
                return False

        else:
            return False
    
    def __abs__(self):

        return vector3(abs(self.x), abs(self.y), abs(self.z))

    def __round__(self, ammt):

        return vector3(round(self.x, ammt), round(self.y, ammt), round(self.z, ammt))

    def norm(self):

        norm = np.sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )
        
        self.x /= norm
        self.y /= norm
        self.z /= norm

        return self
    
    def len(self):

        return np.sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )

    def dir(self):
        return vector3(np.arctan2(self.z, self.y), np.arctan2(self.z, self.x), np.arctan2(self.y, self.x))

    def fromQuaternion(self, quaternion):
        if isinstance(quaternion, Quaternion):

            self.x = quaternion.x
            self.y = quaternion.y
            self.z = quaternion.z

            return self

    def __str__(self):

        return str(self.x) + ', ' + str(self.y) + ', ' + str(self.z)
    
class Quaternion:
    
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):

        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, quaternion):

        if isinstance(quaternion, Quaternion):

            w = self.w + quaternion.w
            x = self.x + quaternion.x
            y = self.y + quaternion.y
            z = self.z + quaternion.z

            return Quaternion(w, x, y, z)
        
        else:

            w = self.w + quaternion
            x = self.x + quaternion
            y = self.y + quaternion
            z = self.z + quaternion

            return Quaternion(w, x, y, z)


    def __sub__(self, quaternion):

        if isinstance(quaternion, Quaternion):

            w = self.w - quaternion.w
            x = self.x - quaternion.x
            y = self.y - quaternion.y
            z = self.z - quaternion.z

            return Quaternion(w, x, y, z)
        
        else:

            w = self.w - quaternion
            x = self.x - quaternion
            y = self.y - quaternion
            z = self.z - quaternion

            return Quaternion(w, x, y, z)

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

    def fractional(self, alpha):

        self.w = 1-alpha + alpha*self.w
        self.x *= alpha
        self.y *= alpha
        self.z *= alpha
        
        return self.norm()

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

    def rotateVector(self, v):

        if isinstance(v, vector3):

            rVector = Quaternion(0.0, 0.0, 0.0, 0.0).fromVector(v)
            rVector = self * rVector * self.conj()

            return vector3(0, 0, 0).fromQuaternion(rVector)
        elif isinstance(v, Quaternion):

            v.w = 0.0
            v = self * v * self.conj()

            return vector3(0, 0, 0).fromQuaternion(v)

        else:

            return TypeError
    
    def fromVector(self, vector):
        if isinstance(vector, vector3):

            self.x = vector.x
            self.y = vector.y
            self.z = vector.z

            return self
        elif isinstance(vector, Quaternion):

            self = vector

            return self
        else:
            raise TypeError
            
class TVC:
    
    def __init__(self):
        
        self.command = vector3()

        self.position = vector3()

        self.Servoposition = vector3()

        self.min = vector3()
        self.max = vector3()

        self.offset = vector3()

        self.noise = vector3()

        self.servoSpeed = 0.0

        self.linkageRatio = vector3()

        self.lever = 0.0

        self.torque = vector3(0.0, 0.0, 0.0)
        self.acceleration = vector3(0.0, 0.0, 0.0)

    def actuate(self, command_angles: vector3, dt):

        self.command = command_angles * RAD_TO_DEG * self.linkageRatio

        errorY = self.command.y - self.Servoposition.y
        errorZ = self.command.z - self.Servoposition.z

        speedY = self.servoSpeed * dt
        speedZ = self.servoSpeed * dt

        errorY = clamp(errorY, -speedY, speedY)
        errorZ = clamp(errorZ, -speedZ, speedZ)

        self.Servoposition.y += errorY
        self.Servoposition.z += errorZ

        self.position.y = ( round(self.Servoposition.y, 0) / self.linkageRatio.y + random.randint(-100, 100) / 100 * self.noise.y) * DEG_TO_RAD
        self.position.z = ( round(self.Servoposition.z, 0) / self.linkageRatio.z + random.randint(-100, 100) / 100 * self.noise.z) * DEG_TO_RAD

        self.position.y += self.offset.y
        self.position.z += self.offset.z

        self.position.y = clamp(self.position.y, self.min.y * DEG_TO_RAD, self.max.y * DEG_TO_RAD)
        self.position.z = clamp(self.position.z, self.min.z * DEG_TO_RAD, self.max.z * DEG_TO_RAD)

    def calculateForces(self, thrust):
        
        self.acceleration.y = np.sin(self.position.y) * thrust
        self.acceleration.z = np.sin(self.position.z) * thrust
        self.acceleration.x = thrust - self.acceleration.y - self.acceleration.z

        self.torque.y = self.acceleration.y * self.lever
        self.torque.z = self.acceleration.z * self.lever

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
        self.localRotationalAcceleration = vector3(0.0, 0.0, 0.0)
        self.rotationalAcceleration = vector3(0.0, 0.0, 0.0)

        self.rotation_euler = vector3(0.0, 0.0, 0.0) # x is roll, y is pitch, z is yaw
        self.rotation_quaternion = Quaternion()
    
    def addLocalForce(self, force):
        self.accelerationInertial += self.rotation_quaternion.rotateVector(force / self.mass)

    def addGlobalForce(self, force):
        self.accelerationInertial += force / self.mass

    def addLocalTorque(self, torque):
        self.addGlobalTorque(self.rotation_quaternion.rotateVector(torque))

    def addGlobalTorque(self, torque):
        self.rotationalAcceleration.x += torque.x / self.mmoi.x
        self.rotationalAcceleration.y += torque.y / self.mmoi.y
        self.rotationalAcceleration.z += torque.z / self.mmoi.z
        

    def update(self, dt):

        self.rotationalVelocity += self.rotationalAcceleration * dt
        
        ang = self.rotationalVelocity.len()

        if ang == 0:
            ang = 1e-5

        self.rotation_quaternion *= Quaternion(0.0, 0.0, 0.0, 0.0).fromAxisAngle(ang*dt, self.rotationalVelocity.x/ang, self.rotationalVelocity.y/ang, self.rotationalVelocity.z/ang)

        self.rotation_euler = self.rotation_quaternion.quaternionToEuler()

        self.accelerationInertial += self.gravity

        self.velocityInertial += self.accelerationInertial * dt
        self.positionInertial += self.velocityInertial * dt

        if self.positionInertial.x <= 0:
            self.velocityInertial = vector3(0.0, 0.0, 0.0)
            self.rotationalVelocity = vector3(0.0, 0.0, 0.0)
            self.positionInertial = vector3(0.0, self.positionInertial.y, self.positionInertial.z)

    def clear(self):
        self.rotationalAcceleration = vector3(0.0, 0.0, 0.0)
        self.accelerationInertial = vector3(0.0, 0.0, 0.0)
        self.accelerationLocal = vector3(0.0, 0.0, 0.0)