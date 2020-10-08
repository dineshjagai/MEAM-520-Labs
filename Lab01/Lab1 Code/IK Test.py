from Calculate_IK import Main
import numpy as np

## Section 1: Four example T0e for testing Calculate_IK
T0e = np.array([0, 0, 1, 292.1, 0, -1 ,0 ,0 ,1, 0, 0, 222.25, 0, 0, 0, 1]).reshape((4, 4))
#T0e = np.array([0, 0.7071, 0.7071, 206.5459, 0, -0.7071 ,0.7071 ,206.5459 ,1, 0, 0, 222.25, 0, 0, 0, 1]).reshape((4, 4))
#T0e = np.array([0, 0.7071, 0.7071, 413.0918, 0, -0.7071 ,0.7071 ,413.0918 ,1, 0, 0, 444.5, 0, 0, 0, 1]).reshape((4, 4))
#T0e = np.array([0.75, 0.3417, 0.5663, 126.483, 0.433, -0.9009,-0.0299 ,73.025 ,0.5, 0.2676, -0.8236, -30.716, 0, 0, 0, 1]).reshape((4, 4))
## Section 1 ends here

## Section 2: This part is for third question in 3.Simulation
# Load targets.npy file to import homogeneous transformation matrix, there are 4 targets you can test
#targets = np.load('targets.npy', allow_pickle=True)

# Choose T0e for the input of IK (uncommand one which you choose)
#T0e = targets[0]         # T0e for the first target
#T0e = targets[1]         # T0e for the second target
#T0e = targets[2]         # T0e for the third target
#T0e = targets[3]         # T0e for the fourth target
## Section 2 ends here

# Please do not modify anything below this
np.set_printoptions(precision=4, suppress=True)
main = Main()
[q, isPos] = main.inverse(T0e)
main.test(T0e, q, isPos)
print('Your homogeneous transformation matrix is \n', T0e)
print('\n')
print('q is\n', q)
print('\n')
print('isPos = \n', isPos)