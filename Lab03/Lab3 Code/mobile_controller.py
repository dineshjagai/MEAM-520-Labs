import numpy as np

class Controller:
    """
    a class implementing path following controller for a differential
    drive robot

    ...

    Attributes
    ----------
    path :
        the desired path to be followed by the robot, an instantiation
        of the Path class
    robot_params : 
        a dictionary of intrinsic robot parameters to be used by
        the controller
    frequency :
        the frequency that the controller loop runs at, measured in Hz
        
    Methods
    -------
    update(x, x_dot):
        calculate the wheel speeds needed to follow the desired path using
        the latest state information
    """

    def __init__(self, path, robot_params, frequency):
        '''
        initialize the Controller class

        Parameters:
            path :
                the desired path for the robot to follow 
                an instance of the Path class
            robot_params:
                a dictionary of intrinsic robot parameters to be used by
                the controller
            frequency :
                the frequency that the controller loop runs at, measured in Hz              
        '''

        # intrinsics
        self.path = path
        self.frequency = frequency # in Hz
        self.r = robot_params['wheel-radius']
        self.d = robot_params['axle-width']
        self.l = robot_params['tracking-point-axle-offset']
        self.integral_e2 = 0


        
    def set_path(self, path):
        self.path = path

    def update(self, x, x_dot):       
        '''
        calculate the wheel speeds needed to follow the desired path using
        the latest state information

        Parameters:
            x :
                numpy array of length 3 representing state of robot
                array elements correspond to [x, y, theta]
            x_dot : 
                numpy array of length 3 representing derivative of robot state
                array elements correspond to [x_dot, y_dot, theta_dot]
                
        Return:
            q_dot : 
                numpy array of length 2 representing desired wheel speeds 
                array elements correspond to [phi_dot_right, phi_dot_left]
            e1 : 
                a measure of position error, a scalar
            e2 : 
                a measure of velocity error, a scalar
        '''
         

############### Start your implementation here ##########################
        
        q_dot = np.array([0, 0])
        e1 = 0
        e2 = 0
        ### STUDENT CODE HERE ###
        kp = 1
        ki = 1
        xp = x[0] + self.l*np.cos(x[-1])
        yp = x[1] + self.l*np.sin(x[-1])
        e1 = self.path.gamma(xp, yp)
        xp_dot = np.array([[1, 0, -self.l*np.sin(x[-1])], [0, 1, self.l*np.cos(x[-1])]])@(x_dot)
        e2 = self.path.v_des - np.sqrt(xp_dot[0]**2 + xp_dot[1]**2)
        delta_t = 1/(self.frequency)
        self.integral_e2 = self.integral_e2 + delta_t*e2
        m_2 = np.array([[-kp*e1], [self.path.v_des + ki*self.integral_e2]])
        jacobian = np.array([[(self.r*np.cos(x[-1])/2 + self.l*self.r*np.sin(x[-1])/self.d), (self.r*np.cos(x[-1])/2 - self.l*self.r*np.sin(x[-1])/self.d)], [
                            (self.r*np.sin(x[-1])/2 - self.l*self.r*np.cos(x[-1])/self.d), (self.r*np.sin(x[-1])/2 + self.l*self.r*np.cos(x[-1])/self.d)]])
        #m_1 = np.array([[self.path.gamma_jacobian(xp, yp)@jacobian], [(self.r/2)*np.array([1, 1])]])
        m_1 = np.vstack((self.path.gamma_jacobian(xp, yp)@jacobian,((self.r/2)*np.array([1, 1]))))
#        print(m_1)
#        print(((self.r/2)*np.array([1, 1])).shape)
#        print((self.path.gamma_jacobian(xp, yp)@jacobian).shape)
        q_dot = np.linalg.inv(m_1)@m_2
        return q_dot, e1, e2