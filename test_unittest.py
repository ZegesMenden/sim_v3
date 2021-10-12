from physics import *
import unittest

class test_vector_math(unittest.TestCase):

    def test_eq(self):
        self.assertEqual( vector3(0.0, 0.0, 0.0), vector3(0.0, 0.0, 0.0) )
    
    def test_add(self):
        self.assertEqual( vector3(0.0, 0.0, 0.0) + vector3(1.0, 1.0, 1.0) , vector3(1.0, 1.0, 1.0) )
    
    def test_sub(self):
        self.assertEqual( vector3(0.0, 0.0, 0.0) - vector3(1.0, 1.0, 1.0), vector3(-1.0, -1.0, -1.0) )

    def test_mul(self):
        self.assertEqual( vector3(1.0, 1.0, 1.0) * vector3(2.0, 2.0, 2.0), vector3(2.0, 2.0, 2.0) )

    def test_div(self):
        self.assertEqual( vector3(1.0, 1.0, 1.0) / vector3(2.0, 2.0, 2.0), vector3(0.5, 0.5, 0.5) )

    def test_norm(self):
        self.assertEqual( vector3(5.0, 5.0, 5.0).norm(), vector3(0.5773502691896257, 0.5773502691896257, 0.5773502691896257) )

    def test_len(self):
        self.assertEqual( vector3(5.0, 5.0, 5.0).len(), 8.660254037844387 )

class test_quaternion_math(unittest.TestCase):

    def test_eq(self):
        self.assertEqual( Quaternion(1.0, 0.0, 0.0, 0.0), Quaternion(1.0, 0.0, 0.0, 0.0) )
    
    def test_add(self):
        self.assertEqual( (Quaternion(0.0, 0.0, 0.0, 0.0) + Quaternion(1.0, 1.0, 1.0, 1.0)).w, Quaternion(1.0, 1.0, 1.0, 1.0).w )

    def test_sub(self):
        self.assertEqual( (Quaternion(1.0, 1.0, 1.0, 1.0) - Quaternion(1.0, 1.0, 1.0, 1.0)).w, Quaternion(0.0, 0.0, 0.0, 0.0).w )

    def test_mul(self):
        self.assertEqual( Quaternion(1.0,0.5,0.5,0.5) * Quaternion(1.0,0.5,0.5,0.5), Quaternion(0.25,1.0,1.0,1.0) )

    def test_conj(self):
        self.assertEqual( Quaternion( 1.0, 0.5, 0.5, 0.5 ).conj(), Quaternion( 1.0, -0.5, -0.5, -0.5 ) )
    
    def test_norm(self):
        self.assertEqual( Quaternion(1.0, 0.5, 0.5, 0.5).norm(), Quaternion(0.75592894602, 0.37796447301, 0.37796447301, 0.37796447301) )

    def test_euler_to_quaternion(self):
        self.assertEqual( Quaternion().eulerToQuaternion(0.174533, 0.174533, 0.174533), Quaternion( 0.9892895259261897, 0.07892647901187541, 0.09406091491321403, 0.07892647901187543 ) )

    def test_quaternion_to_euler(self):
        self.assertEqual( Quaternion( 0.9892895259261897, 0.07892647901187541, 0.09406091491321403, 0.07892647901187543 ).quaternionToEuler(), vector3( 0.174, 0.174, 0.174 ))

if __name__ == '__main__':
    unittest.main()