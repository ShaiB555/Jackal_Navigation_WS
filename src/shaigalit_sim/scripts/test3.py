import rospy

# Initialize the ROS node
rospy.init_node('param_example')

# Retrieve the value of a parameter named 'my_parameter'
param_value = rospy.get_param('at_goal')

# Print the parameter value
print("Parameter value:", param_value)
