import rospy
import numpy as np
import angle_diff

# # Initialize the ROS node
# rospy.init_node('param_example')

# # Retrieve the value of a parameter named 'my_parameter'
# param_value = rospy.get_param('at_goal')

# # Print the parameter value
# print("Parameter value:", param_value)

alpha=1*np.pi
beta=(1/4)*np.pi
print(angle_diff.angle_diff(alpha,beta))

print(np.eye(3))