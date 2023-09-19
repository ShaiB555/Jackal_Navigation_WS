import rospy
import numpy as np
from matplotlib import pyplot as plt

def remove_first_column(matrix):
    return [row[1:] for row in matrix]

if __name__ == '__main__':
       
    
    x_real_vec=rospy.get_param("x_real_vec",[[0.0],[0.0],[0.0]])
    x_est_vec=rospy.get_param("x_est_vec",[[0.0],[0.0],[0.0]])
    time_vec=rospy.get_param("time_vec",[0.0])
    time_vec_real=rospy.get_param("time_vec_real",[0.0])
    x_real_vec=remove_first_column(x_real_vec)
    x_est_vec=remove_first_column(x_est_vec)
    time_vec=time_vec[1:]
    time_vec=np.array(time_vec)-time_vec[0]
    print(time_vec_real)
    time_vec_real=time_vec_real[1:]
    print(time_vec_real)
    time_vec_real=np.array(time_vec_real)-time_vec_real[0]
    plt.figure()
    colors_est=["r-","g-","b-"]
    colors_real=["r--","g--","b--"]
    # print("REAL")
    # print(np.size(x_real_vec))
    # print("EST")
    # print(np.size(x_est_vec)) /// [:len(x_real_vec[i])]
    for i in range(3):
        plt.plot(time_vec,x_est_vec[i],colors_est[i])
        plt.plot(time_vec_real[:len(x_real_vec[i])],x_real_vec[i],colors_real[i])
    plt.xlabel('Time [sec]')
    # plt.legend(["Estimated X Position [m]","Estimated Y Position [m]","Estimated Orientation [rad]"])
    plt.grid()
    plt.show()

    #World map + XY motion
    