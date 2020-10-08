import numpy as np
from math import sin, cos, atan2



def Planner_update(x, start, goal, robot, obstacles):
    """
    This function receives the current robot state, the start and goal positions, 
    and obstacle information. It computes the net force (composition of attractive 
    force and repulsive forces) based on the potential field planner, and it calls
    the select_wheel_vel function to update the wheel velocity command. 

    Inputs:
        x,          current robot state ([x, y, theta], np.array(3))
        start,      start position for one test ([x, y, theta], np.array(3))
        goal,       goal position for one test ([x, y, theta], np.array(3)) 
        robot,      the mobile robot class
        obstacles,  obstacle positions returned by lidar scan ([x, y], np.array(n, 2),
                    n is the number of obstacle points)
    Outputs:
        u,          desired wheel velocities ([right_wheel_vel, left_wheel_vel], np.array(2))
    """
    
    # Initialize net force
    F_net = np.zeros(2)

    # STUDENT CODE STARTS HERE
    # Assume P_0 = inft 

    # Find \rho(O_i(q)), i.e. the shortest distance o_i and any workspace obstacle
    #  Then determine the repulsive force and the attractive force.
    #  Find the sum and return it! 

    eta = 3
    zeta = 9
    min_dist = np.inf
    for obs in obstacles:
        a = (x[0] - obs[0])**2 + (x[1] - obs[1])**2
        dist = np.sqrt(a)        
        if (dist < min_dist):
            min_dist = dist
            clostest_obs = obs
    #  Calculate the gradient for F_rep
    pos = np.array([x[0], x[1]])
    resultant = pos - clostest_obs
    grad_f_rep = (resultant)/(np.linalg.norm(resultant))
    F_rep = eta* (1/min_dist - 0)*(1/(min_dist ** 2))*(grad_f_rep.T)
    
    #  Calculate the attractive force
    goal_pos = np.array([goal[0], goal[1]])
    F_att = -zeta*(pos - goal_pos)

    F_net =  F_att + F_rep
    # Compute wheel velocities from the net force and the current state
    u = select_wheel_vel(F_net, x)
    return u


def select_wheel_vel(F, x):
    """
    This function receives the net force computed by the potential field, and the current 
    robot state. It projects the force onto the robot frame, computes the wheel velocities,
    and returns the velocity command.
    Inputs:
        F,      net force vetor in the potential field, (np.array(2))
        x,      current robot state ([x, y, theta], np.array(3))
       
    Outputs:
        u,      desired wheel velocities ([right_wheel_vel, left_wheel_vel], np.array(2))
    """

    # Initialize wheel speed
    u = np.zeros(2)
    if F[0] == 0 and F[1] == 0:
        return u

    # STUDENT CODE STARTS HERE
    # convert forces to the robot frame to calculate the velocities

    theta = x[2]
    rotation_matrix = np.array([[np.cos(theta), np.sin(theta)],
                  [np.sin(theta), -np.cos(theta)]])
    # F = [[f_t], [f_p]] 
    f = rotation_matrix@F
    print(f)
    # These should scale by the wheel radii
    alpha = 0.23/2
    beta = 0.23/2
    A = np.array([[alpha, alpha],
                  [beta, -beta]])
    u = np.linalg.inv(A)@f
    print(u)
    return u






