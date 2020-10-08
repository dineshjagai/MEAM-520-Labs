import numpy as np


class Path:
    """
    a class defining a path for a mobile robot

    ...

    Attributes
    ----------
    v :
        desired forward velocity to follow the path with


    Methods
    ----------
    gamma : 
        function with arguments x, y that evaluates to 0 when robot is
        on the path
    gamma_jacobian :
        function with arguments x, y that returns and evaluates the 
        jacobian of the gamma function at the point (x, y)
    """

    def __init__(self, v):
        self.v_des = v

    def gamma(self, x, y):
        ### STUDENT CODE HERE ###
        # Test
#        gamma = y
        # 3.1.1a
#        gamma = y -2*x - 1
        # 3.1.1b
#        gamma =  y**2 + x**2 - 1
        # 3.1.1c
#        gamma = y - x**3 +  x**2 + x - 1
        # 3.1.1d
        r = 0.035
        gamma = (y-1)**2 + x**2 - r**2
        return gamma

    def gamma_jacobian(self, x, y):
        # jacobian = np.array([0, 0])
        ### STUDENT CODE HERE ###
        # Test (For line y = 0 )
#        jacobian = np.array([0, 1])
        # 3.1.1a
#        jacobian = np.array([-2, 1])
        # 3.1.1b
#        jacobian = np.array([2*x, 2*y])
        # 3.1.1c
#        jacobian = np.array([-3*(x**2) + 2*x + 1, 1])
        # 3.1.1d
        jacobian = np.array([2*x, 2*(y-1)])

        return jacobian
