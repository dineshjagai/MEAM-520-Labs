import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
import threading
from queue import Queue
import numpy as np
from scipy.spatial.transform import Rotation as R

class ROSInterface:
    """
    a class to manage ROS in a separate thread

    ...

    Attributes
    ----------
    cmd_q :
        a queue for asyncronous communication, holds commands for robot
    scan_q :
        a queue for asyncronous communication, holds scans from robot LIDAR
    state_q :
        a queue for asyncronous communication, holds states of robot
        
    
    Methods
    -------
    reset(x):
        reset the robot to the state x, where x is a numpy array of length 3
    loop():
        function to continously loop and update with latest ROS info
    """

    def __init__(self, cmd_q, scan_q, state_q):
        """
        constructor for the ROSInterface class   
        
        Parameters:    
            cmd_q :
                a queue for asyncronous communication, holds commands for robot
            scan_q :
                a queue for asyncronous communication, holds scans from robot 
            state_q :
                a queue for asyncronous communication, holds states of robot
        """
        self.cmd_q = cmd_q
        self.scan_q = scan_q
        self.state_q = state_q
        self.have_scan = False
        self.have_state = False
        
    def scan_cb(self, scan):
        """
        callback function from scan subscriber
        get the latest scan and put it onto the scan_q
        
        Parameters:
            scan:
                most recent LaserScan msg
        """
        # clear queue
        while not self.scan_q.empty():
            self.scan_q.get()
        self.scan_q.put(scan.ranges)
        self.have_scan = True
        
    def state_cb(self, msg):
        """
        callback function from state subscriber
        get the latest scan and put it onto the state_q
        
        Parameters:
            msg:
                most recent ModelStates msg
        """
        # clear queue
        while not self.state_q.empty():
            self.state_q.get()
        # mobile robot should always be the last object in message
        self.state_q.put({'pose':msg.pose[-1], 'twist':msg.twist[-1]})  
        self.have_state = True

    def send_command(self):
        """
        send twist command to mobile robot from last_cmd attribute
        """
        twist = Twist()
        twist.linear.x = self.last_cmd[0]
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.last_cmd[1]
        self.twist_pub.publish(twist)

    def reset(self, x):
        '''
        resets the robot to a desired state

        Parameters:
            x (np.array(3)): 
                desired state to set robot [x position, y position, theta]
        '''
        state = ModelState()
        state.model_name = 'mobile_base'
        state.pose.position.x = x[0]
        state.pose.position.y = x[1]
        state.pose.position.z = 0
        state.pose.orientation.x = 0
        state.pose.orientation.y = 0
        state.pose.orientation.z = np.sin(x[2] / 2)
        state.pose.orientation.w = np.cos(x[2] / 2)
        rospy.wait_for_service('/gazebo/set_model_state')
        setter = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        setter(state)

    def loop(self):
        '''
        function to indefinitely run in a separate thread
        '''
        rospy.init_node('mobile_controller', disable_signals=True)
        self.twist_pub = rospy.Publisher("/mobile_base/commands/velocity", 
                                         Twist, queue_size=1, latch=True)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.state_sub = rospy.Subscriber("/gazebo/model_states", 
                                          ModelStates, self.state_cb)
        self.last_cmd = [0, 0, 0] 

        rate = rospy.Rate(100)          #poll at 100Hz
        while not rospy.is_shutdown():
            try:
                if self.have_scan and self.have_state:
                    if not self.cmd_q.empty():
                        self.last_cmd = self.cmd_q.get()
                        if self.last_cmd == "stop":
                            break
                    self.send_command()
                else:
                    print("Waiting for Gazebo...")
                rate.sleep()
            except KeyboardInterrupt:
                break

class MobileRobot():
    """
    a class for managing a mobile robot in the gazebo environment

    ...

    Attributes
    ----------
    initial_state :
        numpy array of length 3 describing initial state of robot.
        state corresponds to [x, y, theta]
        optional - if not specified defaults to [0, 0, 0]
        
    Methods
    -------
    set_wheel_velocity(vel):
        take in a list of 2 elements corresponding to the right and left wheel 
        speeds respectively and set the robot's wheel motors to these values
    get_scan():
        use information from the latest LIDAR scan
    get_state():
        read the current state of the robot
    get_params():
        return a dictionary of parameters intrinsic to the mobile robot
    stop():
        gracefully stop the robot, the connection with ROS and the code
    """
    
    def __init__(self, x=np.array([0, 0, 0])):
        '''
        initialize the MobileRobot class

        Parameters:
            initial_state :
                numpy array of length 3 describing initial state of robot.
                state corresponds to [x, y, theta]
                optional - if not specified defaults to [0, 0, 0]
        '''
        self.cur_scan = ()
        self.cur_state = []
        self.cmd_q = Queue()
        self.scan_q = Queue()
        self.state_q = Queue()
        
        # intrinsic properties
        self.r = 0.035
        self.d = 0.23
        self.v_max = 0.7

        #ROS runs in a separate thread because it needs to always
        #check the message buffers
        print("Starting ROS thread...")
        self.ros = ROSInterface(self.cmd_q, self.scan_q, self.state_q)
        self.ros_thread = threading.Thread(target=self.ros.loop)
        self.ros_thread.start()
        print("ROS thread started")
        self.ros.reset(x)
        while self.scan_q.empty() or self.state_q.empty(): pass

    def set_wheel_velocity(self, vel):
        '''
        given an array of 2 values, sets right and left wheel to corresponding
        value

        Parameters:
            vel (np.array(2)): 
                desired robot wheel speeds ([right, left]) in rad/s 
        '''
        if len(vel) == 2:
            vel = np.clip(vel, -self.v_max / self.r, self.v_max / self.r)
            v_f = self.r * np.mean(vel)
            yaw_dot = self.r * (vel[1] - vel[0]) / self.d
            self.cmd_q.put([v_f, yaw_dot])
        else:
            raise Exception("Invalid velocity command")

    def get_scan(self):
        '''
        retreive latest LIDAR scan from robot

        Return:
            scan information 
        '''
        if not self.scan_q.empty():
            self.cur_scan = np.array(self.scan_q.get())
        return self.cur_scan
    
    def get_state(self):
        '''
        retreive latest state information of the robot

        Returns:
            x (np.array(3)):
                state of the robot [x, y, theta]
            x_dot (np.array(3)):
                derivative of the state of the robot [x_dot, y_dot, theta_dot]
        '''
        if not self.state_q.empty():
            state = self.state_q.get()
            quaternion = lambda o: np.array([o.x, o.y, o.z, o.w])
            rot_mat = R.from_quat(quaternion(state['pose'].orientation))
            x = np.array([state['pose'].position.x, state['pose'].position.y,
                          R.as_euler(rot_mat, 'zyx')[0]])
            x_dot = np.array([state['twist'].linear.x, state['twist'].linear.y,
                              state['twist'].angular.z])
            self.cur_state = [x, x_dot]
        return self.cur_state[0], self.cur_state[1]
    
    def get_params(self):
        '''
        retreive dictionary of parameters intrinsic to the robot

        Returns:
            {
                wheel-radius: (m)
                axle-width: (m)
                max-speed: (m/s)
            }
        '''
        return {'wheel-radius': self.r,
                'axle-width': self.d,
                'max-speed': self.v_max}
    
    def stop(self):
        '''
        gracefully stop the robot, the connection with ROS and the code
        '''
        print("Exiting")
        self.cmd_q.put("stop")
        self.ros_thread.join()
