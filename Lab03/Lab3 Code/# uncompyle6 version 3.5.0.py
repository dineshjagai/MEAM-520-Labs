# uncompyle6 version 3.5.0
# Python bytecode 3.6 (3379)
# Decompiled from: Python 2.7.5 (default, Aug  7 2019, 00:51:29) 
# [GCC 4.8.5 20150623 (Red Hat 4.8.5-39)]
# Embedded file name: /home/meam520/code/Lab3/mobile_path.py
# Compiled at: 2020-07-08 02:30:14
# Size of source mod 2**32: 5281 bytes
import numpy as np

class Path:
    r"""'\n    a class defining a path for a mobile robot\n\n    ...\n\n    Attributes\n    ----------\n    v :\n        desired forward velocity to follow the path with\n\n    \n    Methods\n    ----------\n    gamma : \n        function with arguments x, y that evaluates to 0 when robot is\n        on the path\n    gamma_jacobian :\n        function with arguments x, y that returns and evaluates the \n        jacobian of the gamma function at the point (x, y)\n    '"""

    def __init__(self, v):
        self.v_des = v


    def gamma_circle(self, x, y, x_c, y_c, r): 
         (y-y_c)**2 + (x - x_c)**2 - r**2



    def circle(self, x_c, y_c, r):
        """
        set the classes attributes such that the path is a circle
        
        Parameters:    
            x_c :
                desired x coordinate for circle center
            y_c :
                desired y coordinate for circle center
            r :
                desired radius of circle
        """
        def gamma(x,y):
            return (y-y_c)**2 + (x - x_c)**2 - r**2



        self.gamma  = gamma
        self.gamma_jacobian = lambda x, y: np.array([2 * (x - x_c),
         2 * (y - y_c)])

    def y_line(self, m, b):
        """
        set the classes attributes such that the path is a line in form y(x)
        
        Parameters:    
            m :
                the slope of the line
            b : 
                the y intercept of the line
        """
        self.gamma = lambda x, y: y - m * x - b
        self.gamma_jacobian = lambda x, y: np.array([-m, 1])

    def x_line(self, m, b):
        """
        set the classes attributes such that the path is a line in form x(y)
        
        Parameters:    
            m :
                the slope of the line
            b : 
                the x intercept of the line
        """
        self.gamma = lambda x, y: x - m * y - b
        self.gamma_jacobian = lambda x, y: np.array([1, -m])


class Trajectory:
    """"\\n    a class defining a trajectory for a mobile robot, composed of many paths\\n\\n    ...\\n\\n    Attributes\\n    ----------\\n    waypoints :\\n        a numpy array of waypoints of size (N, 2)\\n    v :\\n        desired forward velocity to follow the trajectory\\n    l :\\n        tracking point distance from robot's axle\\n    goal_r :\\n        radius of the goal region for a waypoint (if robot is in this circle \\n        it is considered to be at the waypoint)\\n    goal_theta :\\n        window of acceptable angle error between robot and trajectory (if \\n        robot is outside this window it should try to get point itself back \\n        towards the path)\\n    seg_num :\\n        index of current segment that the robot is following. Segment n is \\n        defined as the path between waypoint n and waypoint n+1\\n    segment :\\n        an instance of the Path class defining the path to follow for the\\n        current segment\\n        \\n    Methods\\n    ----------\\n    update :\\n        using the most recent state information, evaluate which segment the\\n        robot should be on and construct the corresponding Path\\n    \""""

    def __init__(self, x0, waypoints, v, l, goal_r=0.1, goal_theta=np.pi / 2):
        self.waypoints = waypoints
        self.v = v
        self.l = l
        self.goal_r = goal_r
        self.goal_theta = goal_theta
        self.seg_num = 0
        self.update(x0)

    def update(self, x):
        """
        using the most recent state information, evaluate which segment the
        robot should be on and construct the corresponding Path
        
        Parameters:    
            x :
                current state information of the robot (x, y, theta)
        """
        d = np.linalg.norm(x[0:2] - self.waypoints[self.seg_num + 1, :])
        if d < self.goal_r:
            self.seg_num += 1
        if self.seg_num >= len(self.waypoints) - 1:
            path = None
        else:
            path = Path(self.v)
            deltas = self.waypoints[self.seg_num + 1, :] - self.waypoints[self.seg_num, :]
            theta_d = np.arctan2(deltas[1], deltas[0])
            if abs(np.pi - abs(x[2] - theta_d - np.pi)) > self.goal_theta:
                path.circle(x[0], x[1], self.l)
            elif deltas[0] != 0:
                m = deltas[1] / deltas[0]
                b = self.waypoints[(self.seg_num, 1)] - m * self.waypoints[(self.seg_num, 0)]
                path.y_line(m, b)
            else:
                m = deltas[0] / deltas[1]
                b = self.waypoints[(self.seg_num, 0)] - m * self.waypoints[(self.seg_num, 1)]
                path.x_line(m, b)
        self.segment = path