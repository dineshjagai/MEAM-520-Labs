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

        def gamma_jacobian(x,y):
            return np.array([2*(x - x_c), 2*(y-y_c)])

        self.gamma = gamma
        self.gamma_jacobian = gamma_jacobian

        
    def y_line(self, m, b):
        """
        set the classes attributes such that the path is a line in form y(x)
        
        Parameters:    
            m :
                the slope of the line
            b : 
                the y intercept of the line
        """
        def gamma(x,y):
            return y - m*x - b

        def gamma_jacobian(x,y):
            return  np.array([-m, 1])

        self.gamma =  gamma
        self.gamma_jacobian =  gamma_jacobian


    def x_line(self, m, b):
        """
        set the classes attributes such that the path is a line in form x(y)
        
        Parameters:    
            m :
                the slope of the line
            b : 
                the x intercept of the line
        """
        def gamma(x,y):
            return x - m*y - b

        def gamma_jacobian(x,y):
            return  np.array([1, -m])

        self.gamma =  gamma
        self.gamma_jacobian =  gamma_jacobian

    
class Trajectory:
    """
    a class defining a trajectory for a mobile robot, composed of many paths

    ...

    Attributes
    ----------
    waypoints :
        a numpy array of waypoints of size (N, 2)
    v :
        desired forward velocity to follow the trajectory
    l :
        tracking point distance from robot's axle
    goal_r :
        radius of the goal region for a waypoint (if robot is in this circle 
        it is considered to be at the waypoint)
    goal_theta :
        window of acceptable angle error between robot and trajectory (if 
        robot is outside this window it should try to get point itself back 
        towards the path)
    seg_num :
        index of current segment that the robot is following. Segment n is 
        defined as the path between waypoint n and waypoint n+1
    segment :
        an instance of the Path class defining the path to follow for the
        current segment
        
    Methods
    ----------
    update :
        using the most recent state information, evaluate which segment the
        robot should be on and construct the corresponding Path
    """
    def __init__(self, x0, waypoints, v, l, goal_r =0.1, goal_theta=np.pi/2):
        self.waypoints = waypoints
        self.v = v
        self.l = l
        self.goal_r = goal_r
        self.goal_theta = goal_theta

        # initialize segment
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
      

################Start your implementation here##########################
        self.segment = []
        path = Path(self.v)
        # check to see if a path exists (more segments than waypoints \implies no path exists)
        if ((self.waypoints.shape[0]  - 1) <= self.seg_num):
            self.segment = None
            return 
        next_waypoint = self.waypoints[self.seg_num + 1, :]
        # Find the l2 norm between the two consecutive waypoints
        euclid_dist_wps = np.sqrt(np.sum(np.power((x[0:2]- next_waypoint),2)))
        #  Update the segment number if the distance is close ebough to the goal
        if (euclid_dist_wps < self.goal_r):
            self.seg_num  = self.seg_num + 1 
        # If that's not the case, we need to adjust the path so that the robot can move towards the waypoint
        # We shall do this by considering the angles between the robot and the anglers between the next waypoint. 
        # In general, we want the robot to Circle for when the angle error is too high such that it's facing the 
        # next waypoint. Then the robot can move in a straight line to go to the next waypoint.
        else: 
            current_waypoint = self.waypoints[self.seg_num, :]
            next_waypoint = self.waypoints[self.seg_num + 1, :]
            diff = next_waypoint - current_waypoint
            waypoints_angle = math.atan2(diff[1], diff[0])
            angle_error = abs(waypoints_angle - x[2])
            if (abs(angle_error % 2*np.pi) > self.goal_theta):
                path.circle(x[0], x[1], self.l)
                self.segment = path
            else: 
                # x line 
                if (diff[0] == 0):
                    m = 0
                    b = current_waypoint[0]
                    path.x_line(m, b)
                    self.segment = path
                else:
                    m = diff[1] / diff[0]
                    b = current_waypoint[1] - m * current_waypoint[0]
                    path.y_line(m, b)
                    self.segment = path
            








