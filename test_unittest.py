from physics import *
import unittest

class test_vector_math(unittest.TestCase):
    
    def test_add(self):

        self.assertEqual( (vector3(0.0, 0.0, 0.0) + vector3(1.0, 1.0, 1.0)).x , vector3(1.0, 1.0, 1.0).x )
        self.assertEqual( (vector3(0.0, 0.0, 0.0) + vector3(1.0, 1.0, 1.0)).y , vector3(1.0, 1.0, 1.0).y )
        self.assertEqual( (vector3(0.0, 0.0, 0.0) + vector3(1.0, 1.0, 1.0)).z , vector3(1.0, 1.0, 1.0).z )
        
    
    def test_sub(self):

        self.assertEqual( (vector3(0.0, 0.0, 0.0) - vector3(1.0, 1.0, 1.0)).x, vector3(-1.0, -1.0, -1.0).x )
        self.assertEqual( (vector3(0.0, 0.0, 0.0) - vector3(1.0, 1.0, 1.0)).y, vector3(-1.0, -1.0, -1.0).y )
        self.assertEqual( (vector3(0.0, 0.0, 0.0) - vector3(1.0, 1.0, 1.0)).z, vector3(-1.0, -1.0, -1.0).z )

    def test_mul(self):

        self.assertEqual( (vector3(1.0, 1.0, 1.0) * vector3(2.0, 2.0, 2.0)).x, vector3(2.0, 2.0, 2.0).x )
        self.assertEqual( (vector3(1.0, 1.0, 1.0) * vector3(2.0, 2.0, 2.0)).y, vector3(2.0, 2.0, 2.0).y )
        self.assertEqual( (vector3(1.0, 1.0, 1.0) * vector3(2.0, 2.0, 2.0)).z, vector3(2.0, 2.0, 2.0).z )

        self.assertEqual( (vector3(3.0, 3.0, 3.0) * vector3(2.0, 2.0, 2.0)).x, vector3(6.0, 6.0, 6.0).x )
        self.assertEqual( (vector3(3.0, 3.0, 3.0) * vector3(2.0, 2.0, 2.0)).y, vector3(6.0, 6.0, 6.0).y )
        self.assertEqual( (vector3(3.0, 3.0, 3.0) * vector3(2.0, 2.0, 2.0)).z, vector3(6.0, 6.0, 6.0).z )

        self.assertEqual( (vector3(1.0, 1.0, 1.0) * vector3(-2.0, -2.0, -2.0)).x, vector3(-2.0, -2.0, -2.0).x )
        self.assertEqual( (vector3(1.0, 1.0, 1.0) * vector3(-2.0, -2.0, -2.0)).y, vector3(-2.0, -2.0, -2.0).y )
        self.assertEqual( (vector3(1.0, 1.0, 1.0) * vector3(-2.0, -2.0, -2.0)).z, vector3(-2.0, -2.0, -2.0).z )

        self.assertEqual( (vector3(3.0, 3.0, 3.0) * vector3(-2.0, -2.0, -2.0)).x, vector3(-6.0, -6.0, -6.0).x )
        self.assertEqual( (vector3(3.0, 3.0, 3.0) * vector3(-2.0, -2.0, -2.0)).y, vector3(-6.0, -6.0, -6.0).y )
        self.assertEqual( (vector3(3.0, 3.0, 3.0) * vector3(-2.0, -2.0, -2.0)).z, vector3(-6.0, -6.0, -6.0).z )

        self.assertEqual( (vector3(-1.0, -1.0, -1.0) * vector3(2.0, 2.0, 2.0)).x, vector3(-2.0, -2.0, -2.0).x )
        self.assertEqual( (vector3(-1.0, -1.0, -1.0) * vector3(2.0, 2.0, 2.0)).y, vector3(-2.0, -2.0, -2.0).y )
        self.assertEqual( (vector3(-1.0, -1.0, -1.0) * vector3(2.0, 2.0, 2.0)).z, vector3(-2.0, -2.0, -2.0).z )

        self.assertEqual( (vector3(-3.0, -3.0, -3.0) * vector3(2.0, 2.0, 2.0)).x, vector3(-6.0, -6.0, -6.0).x )
        self.assertEqual( (vector3(-3.0, -3.0, -3.0) * vector3(2.0, 2.0, 2.0)).y, vector3(-6.0, -6.0, -6.0).y )
        self.assertEqual( (vector3(-3.0, -3.0, -3.0) * vector3(2.0, 2.0, 2.0)).z, vector3(-6.0, -6.0, -6.0).z )

    def test_div(self):

        self.assertEqual( (vector3(1.0, 1.0, 1.0) / vector3(2.0, 2.0, 2.0)).x, vector3(0.5, 0.5, 0.5).x )
        self.assertEqual( (vector3(1.0, 1.0, 1.0) / vector3(2.0, 2.0, 2.0)).y, vector3(0.5, 0.5, 0.5).y )
        self.assertEqual( (vector3(1.0, 1.0, 1.0) / vector3(2.0, 2.0, 2.0)).z, vector3(0.5, 0.5, 0.5).z )

        self.assertEqual( (vector3(4.0, 4.0, 4.0) / vector3(2.0, 2.0, 2.0)).x, vector3(2.0, 2.0, 2.0).x )
        self.assertEqual( (vector3(4.0, 4.0, 4.0) / vector3(2.0, 2.0, 2.0)).y, vector3(2.0, 2.0, 2.0).y )
        self.assertEqual( (vector3(4.0, 4.0, 4.0) / vector3(2.0, 2.0, 2.0)).z, vector3(2.0, 2.0, 2.0).z )

        self.assertEqual( (vector3(1.0, 1.0, 1.0) / vector3(-2.0, -2.0, -2.0)).x, vector3(-0.5, -0.5, -0.5).x )
        self.assertEqual( (vector3(1.0, 1.0, 1.0) / vector3(-2.0, -2.0, -2.0)).y, vector3(-0.5, -0.5, -0.5).y )
        self.assertEqual( (vector3(1.0, 1.0, 1.0) / vector3(-2.0, -2.0, -2.0)).z, vector3(-0.5, -0.5, -0.5).z )

        self.assertEqual( (vector3(4.0, 4.0, 4.0) / vector3(-2.0, -2.0, -2.0)).x, vector3(-2.0, -2.0, -2.0).x )
        self.assertEqual( (vector3(4.0, 4.0, 4.0) / vector3(-2.0, -2.0, -2.0)).y, vector3(-2.0, -2.0, -2.0).y )
        self.assertEqual( (vector3(4.0, 4.0, 4.0) / vector3(-2.0, -2.0, -2.0)).z, vector3(-2.0, -2.0, -2.0).z )   

        self.assertEqual( (vector3(-1.0, -1.0, -1.0) / vector3(2.0, 2.0, 2.0)).x, vector3(-0.5, -0.5, -0.5).x )
        self.assertEqual( (vector3(-1.0, -1.0, -1.0) / vector3(2.0, 2.0, 2.0)).y, vector3(-0.5, -0.5, -0.5).y )
        self.assertEqual( (vector3(-1.0, -1.0, -1.0) / vector3(2.0, 2.0, 2.0)).z, vector3(-0.5, -0.5, -0.5).z )

        self.assertEqual( (vector3(-4.0, -4.0, -4.0) / vector3(2.0, 2.0, 2.0)).x, vector3(-2.0, -2.0, -2.0).x )
        self.assertEqual( (vector3(-4.0, -4.0, -4.0) / vector3(2.0, 2.0, 2.0)).y, vector3(-2.0, -2.0, -2.0).y )
        self.assertEqual( (vector3(-4.0, -4.0, -4.0) / vector3(2.0, 2.0, 2.0)).z, vector3(-2.0, -2.0, -2.0).z )    

    

if __name__ == '__main__':
    unittest.main()