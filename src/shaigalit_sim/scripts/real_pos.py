#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
import tf
import numpy as np
import time

def model_states_callback(data):
    
    # Generating time vector matching real position data
    current_time = time.time()
    # time_vec_real=rospy.get_param("time_vec_real",[0.0])
    # time_vec_real.append(current_time)
    # rospy.set_param("time_vec_real",time_vec_real)
    
    
    # Find the index of 'jackal' in the model names
    try:
        jackal_index = data.name.index('jackal')
    except ValueError:
        rospy.logwarn("Jackal model not found in ModelStates message.")
        return

    # Get the pose of the 'jackal' model
    jackal_pose = data.pose[jackal_index]

    # Extract orientation as a quaternion
    quaternion = (
        jackal_pose.orientation.x,
        jackal_pose.orientation.y,
        jackal_pose.orientation.z,
        jackal_pose.orientation.w
    )
    # Convert quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll, pitch, yaw = euler

    #Saving the data as a ROS parameter
    # x_real_vec=rospy.get_param("x_real_vec",[[0.0],[0.0],[0.0]])
    x_real = [[jackal_pose.position.x],[jackal_pose.position.y], [yaw]]
    # x_real_vec=np.hstack((x_real_vec,x_real))
    # x_real_vec=x_real_vec.tolist()
    # print(x_real)
    # rospy.set_param("x_real_vec",x_real_vec)
    data_to_save="{:.6f}, {:.6f}, {:.6f}, {:.6f}\n".format(
        current_time,
        jackal_pose.position.x,
        jackal_pose.position.y,
        yaw
    )
    # Open the file in append mode and save the data
    name="real.txt"
    path="/home/shai/Jackal_Navigation_WS/src/shaigalit_sim/Real_Pos/"+name
    with open(path, "a") as file:
        file.write(data_to_save)


    #Now getting the estimated position and time

    current_time=time.time()
    x_est=rospy.get_param("x_est",[[0.0],[0.0],[0.0]])
    x=x_est[0][0]
    y=x_est[1][0]
    yaw=x_est[2][0]
    data_to_save="{:.6f}, {:.6f}, {:.6f}, {:.6f}\n".format(
        current_time,
        x,
        y,
        yaw,
    )
    # Open the file in append mode and save the data
    name="est.txt"
    path="/home/shai/Jackal_Navigation_WS/src/shaigalit_sim/Real_Pos/"+name
    with open(path, "a") as file:
        file.write(data_to_save)

    # # Print the 'jackal.pose' parameter
    # print("Jackal Pose:")
    # print("Position (x, y, z): ({:.2f}, {:.2f}, {:.2f})".format(
    #     jackal_pose.position.x, jackal_pose.position.y, jackal_pose.position.z))
    # print("Yaw: ({:.2f})".format(yaw))


def pos_listener():
        # Initialize ROS node
        rospy.init_node('pos_listener', anonymous=True)

        # Subscribe to the 'gazebo/model_states' topic
        rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

        # Spin to keep the node alive and listen for messages
        rospy.spin()
   

if __name__ == '__main__':
    

    #Creating txt files for the real and estimated positions
    name="real.txt"
    path="/home/shai/Jackal_Navigation_WS/src/shaigalit_sim/Real_Pos/"+name
    with open(path, "w") as file:
        pass
    
    name="est.txt"
    path="/home/shai/Jackal_Navigation_WS/src/shaigalit_sim/Real_Pos/"+name
    with open(path, "w") as file:
        pass

    try:
         pos_listener()
    except rospy.ROSInterruptException:
        pass
