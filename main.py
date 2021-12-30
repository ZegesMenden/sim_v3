import numpy as np
# global x, a, b, u, p, q, r
# x = np.matrix([[0],  # position
#                [0]]) # velocity

# a = np.matrix([[1, 1],
#                [0, 1]])

# b = np.matrix([[1, 0],
#                [0, 0.5]])

# u = np.matrix([[1],  # pos 
#                [0]]) # acc

# p = np.matrix([[0],
#                [0]])

# q = np.matrix([[0], 
#                [0]])

# r = np.matrix([[1], 
#                [1]])

# def update():
#     global x, a, b, u, p, q, r
#     p += q

#     k = p / ( p + r )
#     print(k * u)
#     x = b * ( u * k )
    
#     p = a * p * a + q
#     print(p)

# update()
# # update is x = a*x + b*(uk)

# # def update(self, sensor_reading):
# #     self.p += self.q
# #     k = self.p / ( self.p + self.r )
# #     self.x += k * ( sensor_reading - self.x )
# #     self.p = ( vector3(1, 1, 1) - k ) * self.p

from physics import *

v = vector3(0, 1, 1)

q = Quaternion().eulerToQuaternion(0.0 * DEG_TO_RAD, 0.0, 0.0)

print(v)
print(v.len())
v = q.rotateVector(v)

print(v)
print(v.len())
v = q.conj().rotateVector(v)

print(v)
print(v.len())

import random
good_number = random.randint(1, 10)
won = False

def second():
    if input == (good_number):
        print('you won, play again?(y,n)')
        if input == ('y'):
            main()
        if input == ('n'):
            print('good game')
            return True
    elif input():
        print('you got it wrong')
        return False
        
def main():
    while True:
        print('guess the number, one through ten')
        if second() == True:
            break
        