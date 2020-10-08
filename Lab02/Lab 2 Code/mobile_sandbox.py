from mobile_robot import MobileRobot
from mobile_controller import Controller
from mobile_path import Path
from time import sleep
import matplotlib.pyplot as plt
import numpy as np


if __name__=='__main__':
    
    # Instantiate robot   
    initial_state = np.array([0, 0, np.pi])
    robot = MobileRobot(initial_state)
    
    # Create dictionary of robot parameters
    robot_params = robot.get_params()
    robot_params['tracking-point-axle-offset'] = .1
    
    # Instantiate controller
    controller = Controller(Path(0.2), robot_params, 100)
    
    # Set max time for simulation to run for
    t_max = 30 # seconds
    
    # Initialize data structures for plotting
    cycle = 0
    position = np.empty((t_max * controller.frequency, 2))
    error = np.empty((t_max * controller.frequency, 2))
    
    # Control loop to run until user interrupts it or time expires
    print("Setup complete - entering loop")
    while (cycle / controller.frequency < t_max):
        try:
            # get most recent state
            x, x_dot = robot.get_state()
            position[cycle, :] = x[0:2]
            
            # calculate new wheel speeds
            q_dot, e1, e2 = controller.update(x, x_dot)
            error[cycle, :] = [e1, e2]
            
            # set wheel speeds
            robot.set_wheel_velocity(q_dot)
            
            # repeat this loop after waiting appropriate amount
            sleep(1 / controller.frequency)
            
            # update data structures for plotting
            cycle += 1 
            
        # using CNTRL-C will interrupt loop
        except KeyboardInterrupt:  
            break
    
    # kill simulation
    robot.stop()
    
    # example code to create plots, modify as needed
    fig, ax = plt.subplots()
    t = np.arange(0, cycle) / controller.frequency
    ax.set(xlabel='x (m)', ylabel='y (m)', title='position of robot')
    ax.plot(position[:, 0], position[:, 1])
    ax.grid()
    plt.show()
    
#    E1 plot 
    fig, ax = plt.subplots()
    t = np.arange(0, cycle) / controller.frequency
    ax.set(xlabel='time(s)', ylabel='e1', title='path error (e1)')
    ax.plot(t[:], error[:, 0])
    ax.grid()
    plt.show()
    
    #    E2 plot 
    fig, ax = plt.subplots()
    t = np.arange(0, cycle) / controller.frequency
    ax.set(xlabel='time(s)', ylabel='e2', title='path error (e2)')
    ax.plot(t[:], error[:, 1])
    ax.grid()
    plt.show()