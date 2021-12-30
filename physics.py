from __future__ import annotations
import math
import random
from typing import Callable

DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi

def clamp(n, minn, maxn): # Clamps output to a range
    return max(min(maxn, n), minn)
#thanks orlando

def rotate(x, y, theta) -> vector3: # Rotates a 2D vector by an angle theta, returns a Vector3 with XY and a 0 Z component
    cs = math.cos(theta)
    sn = math.sin(theta)

    rotated_x = x * cs - y * sn
    rotated_y = x * sn + y * cs

    return vector3(rotated_x, rotated_y, 0)
#thanks orlando again

class vector3:

    def __init__(self, x=0.0, y=0.0, z=0.0) -> None:

        """Class representing a 3D vector"""

        self.x = x
        self.y = y
        self.z = z

    def __str__(self) -> str:
        return str(self.x) + ', ' + str(self.y) + ', ' + str(self.z)

    def __neg__(self) -> vector3:
        return vector3(-self.x, -self.y, -self.z)
    
    def __pos__(self) -> vector3:
        return vector3(+self.x, +self.y, +self.z)

    def __add__(self, vector : vector3) -> vector3:

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

    def __sub__(self, vector : vector3) -> vector3:

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

    def __mul__(self, vector : vector3) -> vector3:

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
    
    def __truediv__(self, vector : vector3) -> vector3:
        
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

    def __eq__(self, vector : vector3) -> bool:
        
        if isinstance(vector, vector3):
            
            if self.x == vector.x and self.y == vector.y and self.z == vector.z:
                return True
            else: 
                return False

        else:
            return False

    def __ne__(self, vector : vector3) -> bool:
        
        if isinstance(vector, vector3):
            
            if self.x != vector.x and self.y != vector.y and self.z != vector.z:
                return True
            else: 
                return False

        else:
            return False
    
    def __abs__(self) -> vector3:

        return vector3(abs(self.x), abs(self.y), abs(self.z))

    def __round__(self, ammt) -> vector3:

        return vector3(round(self.x, ammt), round(self.y, ammt), round(self.z, ammt))

    def norm(self) -> float:
        norm = math.sqrt( self.x ** 2 + self.y ** 2 + self.z ** 2 )
        
        if norm == 0.0:
            norm = 0.000000001

        return norm

    def normalize(self) -> vector3:

        norm = self.norm()

        self.x /= norm
        self.y /= norm
        self.z /= norm

        return self

    def dir(self) -> vector3:
        return vector3(math.atan2(self.z, self.y), math.atan2(self.z, self.x), math.atan2(self.y, self.x))

    def dotProduct(self, vector) -> float:
        if isinstance(vector, vector3):
            return ((self.x * vector.x) + (self.y * vector.y) + (self.z * vector.z))
        else: 
            return None

    def crossProduct(self, vector : vector3) -> vector3:

        if isinstance(vector, vector3):
            # ty perry
            return vector3((self.y * vector.z) - (self.z * vector.y), (self.z * vector.x) - (self.x * vector.z), (self.x * vector.y) - (self.y * vector.x))

    def angleBetweenVectors(self, vector : vector3) -> vector3:
        if isinstance(vector, vector3):
            return math.acos(self.dotProduct(vector) / (self.norm() * vector.norm()))
        else:
            return None
            
    def fromQuaternion(self, quaternion : Quaternion) -> vector3:

        if isinstance(quaternion, Quaternion):

            self.x = quaternion.x
            self.y = quaternion.y
            self.z = quaternion.z

            return self
        else:
            return None

class Quaternion:
    
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):

        """A class representing a quaternion"""

        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __eq__(self, quaternion) -> bool:
        
        if isinstance(quaternion, Quaternion):

            if self.w == quaternion.w and self.x == quaternion.x and self.y == quaternion.y and self.z == quaternion.z:
                return True

            else:
                return False
    
    def __ne__(self, quaternion) -> bool:
        
        if isinstance(quaternion, Quaternion):

            if self.w != quaternion.w and self.x != quaternion.x and self.y != quaternion.y and self.z != quaternion.z:
                return True
                
            else:
                return False

    def __str__(self) -> str:
        
        return str(self.w) + ',' + str(self.x) + ',' + str(self.y) + ',' + str(self.z)

    def __add__(self, quaternion) -> Quaternion:

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

    def __sub__(self, quaternion) -> Quaternion:

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

    def __mul__(self, quaternion) -> Quaternion:
        
        qNew = Quaternion(1.0, 0.0, 0.0, 0.0)

        qNew.w = ( self.w * quaternion.w ) - ( self.x * quaternion.x ) - ( self.y * quaternion.y ) - ( self.z * quaternion.z ) # im no betting man but if i were 
        qNew.x = ( self.w * quaternion.x ) + ( self.x * quaternion.w ) + ( self.y * quaternion.z ) - ( self.z * quaternion.y ) # i would bet that at least one 
        qNew.y = ( self.w * quaternion.y ) - ( self.x * quaternion.z ) + ( self.y * quaternion.w ) + ( self.z * quaternion.x ) # of the operations in this function
        qNew.z = ( self.w * quaternion.z ) + ( self.x * quaternion.y ) - ( self.y * quaternion.x ) + ( self.z * quaternion.w ) # is wrong

        # future ZegesMenden here - i was right

        return qNew

    def norm(self) -> Quaternion:

        n = math.sqrt( self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2 )

        self.w /= n
        self.x /= n
        self.y /= n 
        self.z /= n

        return self

    def conj(self) -> Quaternion:

        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def fractional(self, alpha) -> Quaternion:

        self.w = 1-alpha + alpha*self.w
        self.x *= alpha
        self.y *= alpha
        self.z *= alpha
        
        return self.norm()
    
    def eulerToQuaternion(self, roll, pitch, yaw) -> Quaternion:
        
        cr = math.cos( roll / 2 )
        cp = math.cos( pitch / 2 )
        cy = math.cos( yaw / 2 )

        sr = math.sin( roll / 2 )
        sp = math.sin( pitch / 2 )
        sy = math.sin( yaw / 2 )

        self.w = cr * cp * cy + sr * sp * sy
        self.x = sr * cp * cy - cr * sp * sy
        self.y = cr * sp * cy + sr * cp * sy
        self.z = cr * cp * sy - sr * sp * cy

        return self

    def quaternionToEuler(self) -> vector3:

        r = math.atan2( 2.0 * ( self.w * self.x + self.y * self.z ), 1.0 - 2.0 * ( self.x ** 2 + self.y ** 2 ) )
        p = 2.0 * ( self.w * self.y - self.z * self.x )
        y = math.atan2( 2.0 * ( self.w * self.z + self.x * self.y ), 1.0 - 2.0 * ( self.y ** 2 + self.z ** 2 ) )

        return vector3(r, p, y)

    def fromAxisAngle(self, t, x, y, z) -> Quaternion:
        
        sn = math.sin(t / 2.0)

        self.w = math.cos(t / 2.0)
        self.x = x * sn
        self.y = y * sn
        self.z = z * sn

        return self

    def rotation_between_vectors(self, v : vector3) -> vector3:

        qnew : Quaternion = self * Quaternion().fromVector(v)
        qnew.w = 1 - qnew.w

        return qnew.norm()

    def rotateVector(self, v) -> vector3:

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
    
    def fromVector(self, vector) -> Quaternion:

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

        self.force = vector3(0.0, 0.0, 0.0)

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
        
        self.force.y = math.sin(self.position.y) * thrust
        self.force.z = math.sin(self.position.z) * thrust
        self.force.x = thrust - self.force.y - self.force.z

class DOF6:

    def __init__(self) -> None:

        # aero things

        self.aoa = 0.0

        self.useAerodynamics = False

        self.dragArea = 0.0
        self.dragCoefficient = 0.0

        self.aoa = 0.0
        self.cpLocation = vector3()

        self.wind = vector3()

        self.airDensity = 1.225

        self.dragForce = vector3()

        # stops rocket from rotating on the pad (below 0.01m AGL)
        self.padClamp = True

        # not aero things

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
    
    def addForce(self, force : vector3) -> None:

        if isinstance(force, vector3):

            self.accelerationInertial += force / self.mass

        else:

            raise TypeError

    def addLocalForce(self, force : vector3) -> None:

        if isinstance(force, vector3):

            self.addForce(self.rotation_quaternion.rotateVector(force))

        else:

            raise TypeError

    def addTorque(self, torque : vector3) -> None:

        if isinstance(torque, vector3):

            self.rotationalAcceleration += vector3(0.0, torque.y / self.mmoi.y, torque.z / self.mmoi.z)

        else:
            raise TypeError

    def addTorqueLocal(self, torque : vector3) -> None:

        if isinstance(torque, vector3):

            globalTorque = self.rotation_quaternion.rotateVector(torque)
            self.addTorque(globalTorque)
        
        else:
            raise TypeError

    def applyForce(self, force : vector3, displacement : vector3) -> None:

        if isinstance(force, vector3) and isinstance(displacement, vector3):

            torque = displacement.crossProduct(force)
            accel = force / self.mass

            self.addTorque(torque)
            self.accelerationInertial += accel

        else:
            raise TypeError

    def applyLocalForce(self, force : vector3, displacement : vector3) -> None:

        if isinstance(force, vector3) and isinstance(displacement, vector3):

            gForce = self.rotation_quaternion.rotateVector(force)
            gPos = self.rotation_quaternion.rotateVector(displacement)

            self.applyForce(gForce, gPos)

        else:
            raise TypeError

    def calaulateAerodynamics(self) -> None:

        if self.velocityInertial.x != 0.0:

            velRelative = self.velocityInertial - self.wind

            # velRelative.x = 0.0

            rotation_pogchamp = self.rotation_quaternion.rotateVector(vector3(1.0, 0.0, 0.0))

            self.aoa = velRelative.angleBetweenVectors(rotation_pogchamp)

            dc = self.dragCoefficient * self.aoa
            
            if self.positionInertial.x != 0.0:
                self.dragForce = -velRelative.normalize() * 0.5 * self.airDensity * ( self.velocityInertial.norm() ** 2 ) * dc * self.dragArea
            else:
                self.dragForce = vector3()

    def update(self, dt) -> None:

        self.rotationalVelocity += self.rotationalAcceleration * dt
        
        ang = self.rotationalVelocity.norm()

        if ang == 0:
            ang = 1e-5

        self.rotation_quaternion *= Quaternion(0.0, 0.0, 0.0, 0.0).fromAxisAngle(ang*dt, self.rotationalVelocity.x/ang, self.rotationalVelocity.y/ang, self.rotationalVelocity.z/ang)

        self.rotation_euler = self.rotation_quaternion.quaternionToEuler()

        self.accelerationLocal = self.rotation_quaternion.conj().rotateVector(self.accelerationInertial)

        self.accelerationInertial += self.gravity

        self.velocityInertial += self.accelerationInertial * dt
        self.positionInertial += self.velocityInertial * dt

        if self.positionInertial.x <= 0:
            self.velocityInertial = vector3(0.0, 0.0, 0.0)
            self.rotationalVelocity = vector3(0.0, 0.0, 0.0)
            self.positionInertial = vector3(0.0, self.positionInertial.y, self.positionInertial.z)

    def clear(self) -> None:
        self.rotationalAcceleration = vector3(0.0, 0.0, 0.0)
        self.accelerationInertial = vector3(0.0, 0.0, 0.0)
        self.accelerationLocal = vector3(0.0, 0.0, 0.0)

# dont mind this

# x  = 0.81 * 9.8 * (1.279 ** 2) * (0.3 ** 2)
# x /= 4 * (math.pi * math.pi) * 0.54
# print(x)