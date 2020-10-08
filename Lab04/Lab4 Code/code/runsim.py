"""
Potential Field Planner: Main Simulation
@author: meam520
"""

from mobile_robot import MobileRobot
from potential_field import Planner_update
from lidar_scan import lidar_scan

from time import sleep
import matplotlib.pyplot as plt
import numpy as np


if __name__=='__main__':	
    
    # Simulation setup
    # Modify this part for your tests
    start =   [4, -3, 0]	      # [x, y, theta], coordinates of start, default: [0, 0, 0]
    goal = [0 ,0, 0]       # [x, y, theta], coordinates of goal
    
    print("Setup complete")
    print("Start:", start)
    print("Goal:", goal)
    
    robot = MobileRobot(start)
    
    # Simulation loop
    Done = 0
    goal_x = goal[0]
    goal_y = goal[1]
    
    while not Done:
        try:
            
            # Retrieve current robot state
            x, x_dot = robot.get_state()
            current_x = x[0]
            current_y = x[1]
            print("Current position:", x)

            # Update Lidar scan for obstacles
            obstacles = lidar_scan(robot, x)
    
            # Plan the desired robot velocity using potential field
            wheel_vel = Planner_update(x, start, goal, robot, obstacles)
             
            # Set desired robot velocity
            robot.set_wheel_velocity(wheel_vel)
            # print("Wheel speed:", wheel_vel)
            
            # Modify the goal-reaching conditions if needed
            if abs(goal_x-current_x) <= 0.5 and abs(goal_y-current_y) <= 0.5:
                Done = 1
            
            sleep(0.01)
            
        except (KeyboardInterrupt, ValueError):
        #except Exception as e:
         #   print(e)
            break
    	
    # Stop the robot
    robot.stop()
  

        
      
        
     
    
