from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt

def lidar_scan(robot, x, max_dis = 5):
    """
    This function gets one scan from LIDAR, and transfers the data into positions
    of the point cloud. It also detects whether collision happens.
    Inputs:
        robot,      the mobile robot class
        x,          current robot state ([x, y, theta], np.array(3))
        max_dis,    (optional) maximum distance of the scan range, default is 5 m
       
    Outputs:
        obstacles,  obstacle positions returned by lidar scan ([x, y], np.array(n, 2),
                    n is the number of obstacle points)
    """
    
    # Get scan updates from lidar
    scan = robot.get_scan()
    n = scan.shape[0]

    # Uncomment the following lines to show the scan result in a plot
    robot.set_wheel_velocity([0, 0])
    ax = plt.subplot(111, projection='polar')
    ax.scatter(np.linspace(0, 2*np.pi, n), scan)
    ax.set_rlim([0, 5])
    plt.show()

    # Do not modify code below
    # Get robot dimension
    r = robot.get_params()['axle-width']/2

    # Calculate obstacle coordinates (x, y) and detect collision
    obs_list = []
    for i in range(n):
        if scan[i] < max_dis:
            theta = x[2] + 2 * np.pi * i/n + np.pi
            obs_x = x[0] + cos(theta)*scan[i]
            obs_y = x[1] + sin(theta)*scan[i]
            obs_list.append([obs_x, obs_y])
        if scan[i] < r + 0.05:
            print("Failure: robot collided with obstacles")
            raise ValueError

    # Reshape to a m*2 numpy array
    obstacles = np.array(obs_list)
    
    return obstacles