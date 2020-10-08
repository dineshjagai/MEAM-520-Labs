import numpy as np
import math

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

	def test(self, T0e, q, isPos):
		"""
		Added a test function 

		"""
		T0e_q_intial = np.array([0, 0, 1, 292.1, 0, -1 ,0 ,0 ,1, 0, 0, 222.25, 0, 0, 0, 1]).reshape((4, 4))
		T0e_q_1 = np.array([0, 0.7071, 0.7071, 206.5459, 0, -0.7071 ,0.7071 ,206.5459 ,1, 0, 0, 222.25, 0, 0, 0, 1]).reshape((4, 4))
		T0e_q_2 = np.array([0, 0.7071, 0.7071, 413.0918, 0, -0.7071 ,0.7071 ,413.0918 ,1, 0, 0, 444.5, 0, 0, 0, 1]).reshape((4, 4))
		T0e_q_3 = np.array([0.75, 0.3417, 0.5663, 126.483, 0.433, -0.9009,-0.0299 ,73.025 ,0.5, 0.2676, -0.8236, -30.716, 0, 0, 0, 1]).reshape((4, 4))

		if ((T0e == T0e_q_intial).all()):
			try: 
				assert (len(q) == 5)
				assert (isPos == 1) 
				assert (q[0] == 0)
				assert (q[1] == 0)
				assert (q[2] == 0)
				assert (q[3] == 0)
				assert (q[4] == 0)
			except AssertionError as e:
				raise( AssertionError( "----Error with %s. %s----" % (T0e_q_intial,e)))
		elif (((T0e == T0e_q_1).all())):
			try: 
				assert (len(q) == 5)
				assert (isPos == 1) 
				assert (q[0] == 0.7854)
				assert (q[1] == 0)
				assert (q[2] == 0)
				assert (q[3] == 0)
				assert (q[4] == 0)
			except AssertionError as e:
				raise( AssertionError( "----Error with %s. %s----" % (T0e_q_1,e)))
		elif (((T0e == T0e_q_2).all())):
			try: 
				assert (len(q) == 0)
				assert (isPos == 0) 
			except AssertionError as e:
				raise( AssertionError( "----Error with %s. %s----" % (T0e_q_2,e)))
		elif (((T0e == T0e_q_3).all())):
			try: 
				assert (len(q) == 5)
				assert (isPos == 1) 
				assert (q[0] == 0.5236)
				assert (q[1] == 0)
				assert (q[2] == 1.0472)
				assert (q[3] == 0)
				assert (q[4] == 0)
			except AssertionError as e:
				raise( AssertionError( "----Error with %s. %s----" % (T0e_q_3,e)))
		else :
			print("Erroneous T0e")



	def inverse(self, T0e):


		"""
		INPUT:
		T - 4 x 4 homogeneous transformation matrix, representing 
		 the end effector frame expressed in the base (0) frame
		 (position in mm)

		OUTPUT:
		q - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) 
		 which are required for the Lynx robot to reach the given 
		 transformation matrix T. Each row represents a single
		 solution to the IK problem. If the transform is
		 infeasible, q should be all zeros. 
		isPos - a boolean set to true if the provided
			 transformation T is achievable by the Lynx robot as given,
			 ignoring joint limits
		"""
		
		isPos = 1
		q = np.zeros((5, 1))
		# print(q)
		
		# # Your code ends here
		
		return q, isPos
