from mobile_robot import MobileRobot
from mobile_controller import Controller
from mobile_path import Path, Trajectory
from Astar import Astar
from RRT import RRT
from OccupancyMap import OccupancyMap
from time import sleep, time
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import numpy as np


if __name__=='__main__':
    
    # Parameters defining simulation
    initial_state = np.array([0, 0, 0]) # (x, y, theta)
    start_state = np.array([0, 0]) # (x, y) 
    goal_state = np.array([9, 9]) # (x, y) 
    l = .1 # m
    v = .4 # m/s
    t_max = 30 # seconds
    world = 'tests/map1.json'
    
    # Instantiate robot   
    robot = MobileRobot(initial_state)

    # Create dictionary of robot parameters
    robot_params = robot.get_params()
    robot_params['tracking-point-axle-offset'] = l
    
    # get info about the robot world
    occ_map = OccupancyMap(world, robot_params['axle-width'] / 2)
    robot.set_world(occ_map)

    # Create trajectory
    print("Planning trajetory...")
    t0 = time()
    waypoints = Astar(world, start_state, goal_state, 
                      robot_params['axle-width'] / 2)
    #waypoints = RRT(world, start_state, goal_state, 
    #                robot_params['axle-width'] / 2)
    t1 = time()
    print("Planning completed in {} seconds".format(np.round(t1 - t0, 2)))
    trajectory = Trajectory(initial_state, waypoints, v, l)

    # Instantiate controller
    controller = Controller(trajectory.segment, robot_params, 50)
    
    # Initialize data structures for plotting
    states = np.empty((t_max * controller.frequency, 3))
    error = np.empty((t_max * controller.frequency, 2))
    
    # Control loop to run until user interrupts it or time expires
    print("Setup complete. Entering loop.")
    cycle = 0
    while (cycle / controller.frequency < t_max):
        try:
            # get most recent state information
            x, x_dot = robot.get_state()
            
            # get trajectory update
            trajectory.update(x)
            if trajectory.segment is None:
                break
            else:
                controller.set_path(trajectory.segment)
            
            # calculate and set new wheel speeds
            q_dot, e1, e2 = controller.update(x, x_dot)
            robot.set_wheel_velocity(q_dot)
                        
            # update data structures for plotting
            states[cycle, :] = x
            error[cycle, :] = [e1, e2]
            
            # repeat this loop after waiting appropriate amount
            sleep(1 / controller.frequency)
            cycle += 1 

        # using CNTRL-C or the robot crashing will interrupt loop
        except (KeyboardInterrupt, ValueError):  
            break
    
    # kill simulation
    robot.stop()
    
    # trim data structures
    states = states[:cycle, :]
    error = error[:cycle, :]
    

    fig, ax = plt.subplots()
    t = np.arange(0, cycle) / controller.frequency
    ax.set(xlabel='x (m)', ylabel='y (m)', title='Position of Robot')
    ax.plot(states[:, 0], states[:, 1], color='blue', linewidth=3, label='Actual Trajectory')
    ax.plot(waypoints[:,0], waypoints[:,1], color='orange', linewidth=1, label='Planned Trajectory')
    ax.grid()
    with open(world) as file:
        for block in occ_map.get_blocks():
            obstacle = Rectangle((block[0], block[2]),  np.diff(block[0:2]),
                                 np.diff(block[2:]), color='red', fill=True)
            ax.add_patch(obstacle)
    ax.legend()
    ax.grid()
    with open(world) as file:
        for block in occ_map.get_blocks():
            obstacle = Rectangle((block[0], block[2]),  np.diff(block[0:2]), 
                                 np.diff(block[2:]), color='black', fill=True)
            ax.add_patch(obstacle)
    plt.xlim(occ_map.bounds[0], occ_map.bounds[1])
    plt.ylim(occ_map.bounds[2], occ_map.bounds[3])


    ### ADD MORE PLOTS HERE AS NEEDED ###
    
    # Show all plots
    plt.show()
