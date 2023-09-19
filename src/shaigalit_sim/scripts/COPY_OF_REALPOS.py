#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
import numpy as np
import tf
import time


def model_states_callback(data):

    #Generating time vector matching real position data
    current_time = time.time()
    time_vec_real=rospy.get_param("time_vec_real",[0.0])
    time_vec_real.append(current_time)
    rospy.set_param("time_vec_real",time_vec_real)
    
    #Saving real position data
    model_names = data.name
    jackal_index = model_names.index('jackal')  # 'jackal' is the model name in Gazebo
    robot_pose = data.pose[jackal_index]
    x_real_vec=rospy.get_param("x_real_vec",[[0.0],[0.0],[0.0]])
    x = robot_pose.position.x
    y = robot_pose.position.y
    
    # Extract orientation as a quaternion
    quaternion = (
        robot_pose.orientation.x,
        robot_pose.orientation.y,
        robot_pose.orientation.z,
        robot_pose.orientation.w
    )
    # Convert quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll, pitch, yaw = euler

    #Appending new state to saved vector
    x_real = [[x], [y], [yaw]]
    # print(x_real)
    x_real_vec=np.hstack((x_real_vec,x_real))
    x_real_vec=x_real_vec.tolist()
    rospy.set_param("x_real_vec",x_real_vec)

if __name__ == '__main__':
    # Initialize ROS node for real position and control
    rospy.init_node('gazebo_pos_listener', anonymous=True)
    # Subscribe to the model_states topic
    rospy.set_param("time_vec_real",[0.0])
    rospy.set_param("x_real_vec",[[0.0],[0.0],[0.0]])
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    rospy.spin()