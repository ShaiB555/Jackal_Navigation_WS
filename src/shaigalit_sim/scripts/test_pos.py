import rospy
from gazebo_msgs.msg import ModelStates
import tf
import numpy as np

def model_states_callback(data):
    # Find the index of 'jackal' in the model names
    try:
        jackal_index = data.name.index('jackal')
    except ValueError:
        rospy.logwarn("Jackal model not found in ModelStates message.")
        return

    # Get the pose of the 'jackal' model
    jackal_pose = data.pose[jackal_index]
    x_real_vec=rospy.get_param("x_real_vec",[[0.0],[0.0],[0.0]])

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

    x_real = [[jackal_pose.orientation.x],[jackal_pose.orientation.y], [yaw]]
    print(x_real)
    x_real_vec=np.hstack((x_real_vec,x_real))
    x_real_vec=x_real_vec.tolist()
    rospy.set_param("x_real_vec",x_real_vec)

    # # Print the 'jackal.pose' parameter
    # print("Jackal Pose:")
    # print("Position (x, y, z): ({:.2f}, {:.2f}, {:.2f})".format(
    #     jackal_pose.position.x, jackal_pose.position.y, jackal_pose.position.z))
    # print("Yaw: ({:.2f})".format(yaw))

if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('pose_echo', anonymous=True)

        # Subscribe to the 'gazebo/model_states' topic
        rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

        # Spin to keep the node alive and listen for messages
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
