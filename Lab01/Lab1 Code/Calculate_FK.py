import numpy as np

class Main():
    
    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable
        
        """
        # Lynx ADL5 constants in mm
        self.d1 = 76.2                      # Distance between joint 0 and joint 1
        self.a2 = 146.05                    # Distance between joint 1 and joint 2
        self.a3 = 187.325                   # Distance between joint 2 and joint 3
        self.d4 = 34                        # Distance between joint 3 and joint 4
        self.d5 = 76.2                      # Distance between joint 3 and joint 5
        self.lg = 28.575                    # Distance between joint 5 and end effector (gripper length)
        
        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)
    
    def forward(self, q):
        """
        INPUT:
        q - 1x6 vector of joint inputs [q0,q1,q2,q3,q4,lg]
        
        OUTPUTS:
        jointPositions - 6 x 3 matrix, where each row represents one 
                  joint along the robot. Each row contains the [x,y,z]
                  coordinates of the respective joint's center (mm). For
                  consistency, the first joint should be located at 
                  [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix, 
                  representing the end effector frame expressed in the 
                  base (0) frame
        """
        # Your code starts from here        
        T_0_1 = np.array([[np.cos(q[0]), 0, -1*np.sin(q[0]), 0],
                         [np.sin(q[0]), 0, np.cos(q[0]), 0],
                         [0, -1, 0, self.d1],
                         [0, 0, 0, 1]])
        
        T_1_2 = np.array([[np.sin(q[1]), np.cos(q[1]), 0, self.a2*np.sin(q[1])],
                         [-1*np.cos(q[1]), np.sin(q[1]), 0, -1*self.a2*np.cos(q[1])],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        
        T_2_3 = np.array([[-1*np.sin(q[2]), -1*np.cos(q[2]), 0, -1*self.a3*np.sin(q[2])],
                         [np.cos(q[2]), -1*np.sin(q[2]), 0, self.a3*np.cos(q[2])],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
        
        T_3_4 = np.array([[np.sin(q[3]), 0, np.cos(q[3]), 0],
                         [-1*np.cos(q[3]), 0, np.sin(q[3]), 0],
                         [0, -1, 0, 0],
                         [0, 0, 0, 1]])
        
        T_4_5 = np.array([[np.cos(q[4]), -1*np.sin(q[4]), 0, 0],
                         [np.sin(q[4]), np.cos(q[4]), 0, 0],
                         [0, 0, 1, self.d5],
                         [0, 0, 0, 1]])
        
        T_5_6 = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, self.lg],
                         [0, 0, 0, 1]])
        
        P = np.array([[0],[0],[0],[1]])
        P_1 = T_0_1@P
        P_2 = T_0_1@T_1_2@P
        P_3 = T_0_1@T_1_2@T_2_3@P
        P_4 = T_0_1@T_1_2@T_2_3@T_3_4@(np.array([[self.d4*np.sin(q[3])],[0],[self.d4*np.cos(q[3])],[1]]))
        P_5 = T_0_1@T_1_2@T_2_3@T_3_4@T_4_5@P
        # print(P)
        # print(P_1)
        # print(P_2)
        # print(P_3)
        # print(P_4)
        # print(P_5)
        
        jointPositions = np.vstack((np.delete(P.T,3), 
                                    np.delete(P_1.T,3),
                                    np.delete(P_2.T,3),
                                    np.delete(P_3.T,3),
                                    np.delete(P_4.T,3),
                                    np.delete(P_5.T,3)))
        T0e = T_0_1@T_1_2@T_2_3@T_3_4@T_4_5@T_5_6
        # Your code ends here
        
        return jointPositions, T0e