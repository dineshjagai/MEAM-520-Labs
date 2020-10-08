from arm_controller import ArmController
from time import sleep
import numpy as np

q = [0,0,0,0,0,0]
#q = [0.1,0.1,0.1,0.1,0.1,0.1]
#q = [0,-np.pi/4,np.pi/3,-np.pi/2,0,0]
#q = [0,0,0,10,0,0]
    
if __name__=='__main__':
    cont = ArmController()
    sleep(1)
    print("Setup complete")
    cont.set_state(q)
    sleep(3)
    print('The current state is', cont.get_state())    
    cont.stop()