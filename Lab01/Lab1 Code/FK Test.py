from Calculate_FK import Main
import numpy as np

## Section 1: Four example q for testing Calculate_FK
q = [0, 0, 0, 0, 0, 0]
#q = [3.14, 0, 0, 0, 0, 0]
#q = [0, 1.57, 0, 0, 0, 0]
#q = [0, 0, -1.05, 0, 0, 0]
## Section 1 ends here

## Section 2
# Modify q for the input of FK
#q = [0,0,0,0,0,0]

## Section 2 ends here


# Please do not modify anything below this
np.set_printoptions(precision=3, suppress=True)
main = Main()
[jointPostions, T0e] = main.forward(q)
print('Your q is \n', q)
print('\n')
print('The jointPositions are\n', jointPostions)
print('\n')
print('The homogeneous transformation matrix is\n', T0e)