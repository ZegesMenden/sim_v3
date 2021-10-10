import numpy as np

DEG_TO_RAD = np.pi / 180
RAD_TO_DEG = 180 / np.pi

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

    def __div__(self, vector):
        
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
    
class Quaternion:
    
    def __init__(self, w, x, y, z):

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

    def __mul__(self, quaternion):
        
        qNew = Quaternion(1.0, 0.0, 0.0, 0.0)

        qNew.w = ( self.w * quaternion.w ) - ( self.x * quaternion.x ) - ( self.y * quaternion.y ) - ( self.z * quaternion.z ) # im no betting man but if i were 
        qNew.x = ( self.w * quaternion.x ) - ( self.x * quaternion.w ) - ( self.y * quaternion.z ) - ( self.z * quaternion.y ) # i would bet that at least one 
        qNew.y = ( self.w * quaternion.y ) - ( self.x * quaternion.z ) - ( self.y * quaternion.w ) - ( self.z * quaternion.x ) # of the operations in this function
        qNew.z = ( self.w * quaternion.z ) - ( self.x * quaternion.y ) - ( self.y * quaternion.x ) - ( self.z * quaternion.w ) # is wrong

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

    def rotateVector(self, vector):

        if isinstance(vector, vector3):

            rVector = Quaternion(0.0, vector.x, vector.y, vector.z)
            rVector = self * rVector * self.conj()

            return vector3(rVector.x, rVector.y, rVector.z)