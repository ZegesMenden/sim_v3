from physics import *
import unittest

class test_vector_math(unittest.TestCase):
    
    def test_add(self):

        self.assertEqual( vector3(0.0, 0.0, 0.0) + vector3(1.0, 1.0, 1.0) , vector3(1.0, 1.0, 1.0) )
    
    def test_sub(self):

        self.assertEqual( vector3(0.0, 0.0, 0.0) - vector3(1.0, 1.0, 1.0), vector3(-1.0, -1.0, -1.0) )

    def test_mul(self):

        self.assertEqual( vector3(1.0, 1.0, 1.0) * vector3(2.0, 2.0, 2.0), vector3(2.0, 2.0, 2.0) )
        self.assertEqual( vector3(1.0, 1.0, 1.0) * vector3(-2.0, -2.0, -2.0), vector3(-2.0, -2.0, -2.0) )
        self.assertEqual( vector3(-1.0, -1.0, -1.0) * vector3(2.0, 2.0, 2.0), vector3(-2.0, -2.0, -2.0) )
        self.assertEqual( vector3(-1.0, -1.0, -1.0) * vector3(-2.0, -2.0, -2.0), vector3(2.0, 2.0, 2.0) )

    def test_div(self):

        self.assertEqual( vector3(1.0, 1.0, 1.0) / vector3(2.0, 2.0, 2.0), vector3(0.5, 0.5, 0.5) )
        self.assertEqual( vector3(1.0, 1.0, 1.0) / vector3(-2.0, -2.0, -2.0), vector3(-0.5, -0.5, -0.5) )
        self.assertEqual( vector3(-1.0, -1.0, -1.0) / vector3(2.0, 2.0, 2.0), vector3(-0.5, -0.5, -0.5) )
        self.assertEqual( vector3(-1.0, -1.0, -1.0) / vector3(-2.0, -2.0, -2.0), vector3(0.5, 0.5, 0.5) )

# class test_quaternion_math(unittest.TestCase):


if __name__ == '__main__':
    unittest.main()