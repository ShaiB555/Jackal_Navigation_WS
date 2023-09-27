import rospy
import numpy as np
from matplotlib import pyplot as plt

def remove_first_column(matrix):
    return [row[1:] for row in matrix]

def extract_first_elements(arr, m):
    if m <= 0:
        return []  # Return an empty list if m is not a positive integer

    return [arr[i] for i in range(0, len(arr), m)]

if __name__ == '__main__':

    print("START")

    # Specify the path to the CSV file
    path_real="/home/shai/Jackal_Navigation_WS/src/shaigalit_sim/Real_Pos/real.txt"
    path_est="/home/shai/Jackal_Navigation_WS/src/shaigalit_sim/Real_Pos/est.txt"
    
    # Initialize empty lists to store the data
    real_time_list = []
    x_list = []
    y_list = []
    yaw_list = []

    # Open the CSV file for reading
    with open(path_real, "r") as file:
        # Iterate through each line in the file
        for line in file:
            # Split the line into individual values using a comma as the delimiter
            values = line.strip().split(",")

            # Check if there are enough values on the line (at least 4)
            if len(values) >= 4:
                try:
                    # Parse the values and append them to their respective lists
                    real_time_list.append(float(values[0]))
                    x_list.append(float(values[1]))
                    y_list.append(float(values[2]))
                    yaw_list.append(float(values[3]))
                except ValueError:
                    # Handle any parsing errors
                    print(f"Skipping line: {line}")


    print("MIDDLE")

    # Now, current_time_list, x_list, y_list, and yaw_list contain the extracted data
    # You can use these lists as needed in your code
   
    x_real_list=[[x_list],[y_list],[yaw_list]]
    
    # Initialize empty lists to store the data
    est_time_list = []
    est_x_list = []
    est_y_list = []
    est_yaw_list = []

    # Open the CSV file for reading
    with open(path_est, "r") as file:
        # Iterate through each line in the file
        for line in file:
            # Split the line into individual values using a comma as the delimiter
            values = line.strip().split(",")

            # Check if there are enough values on the line (at least 4)
            if len(values) >= 4:
                try:
                    # Parse the values and append them to their respective lists
                    est_time_list.append(float(values[0]))
                    est_x_list.append(float(values[1]))
                    est_y_list.append(float(values[2]))
                    est_yaw_list.append(float(values[3]))
                except ValueError:
                    # Handle any parsing errors
                    print(f"Skipping line: {line}")

    # Now, current_time_list, x_list, y_list, and yaw_list contain the extracted data
    # You can use these lists as needed in your code
   
    print("END")
    x_est_list=[[est_x_list],[est_y_list],[est_yaw_list]]
    
    # est_time_list=est_time_list[1:]
    est_time_list=np.array(est_time_list)-est_time_list[0]
    # real_time_list=real_time_list[1:]
    real_time_list=np.array(real_time_list)-real_time_list[0]
    colors_est=["r-","g-","b-"]
    colors_real=["r--","g--","b--"]
    plt.figure()
    for j in range(3):
        plt.plot(real_time_list,x_real_list[j][0][:],colors_real[j])
        plt.plot(est_time_list,x_est_list[j][0][:],colors_est[j])
    plt.xlabel('Time [sec]')
    plt.legend(["X_real [m]","X_est [m]","Y_real [m]", "Y_est [m]","Phi_real [rad]","Phi_est [rad]"])
    # plt.legend(["Estimated X Position [m]","Estimated Y Position [m]","Estimated Orientation [rad]"])
    plt.ylabel("State Vector Value")
    plt.grid()
    plt.show()


    #error plot
    plt.figure()
    # x_est_list=remove_first_column(x_est_list)
    error=np.array(x_est_list)-np.array(x_real_list)
    error.tolist()
    for j in range(3):
        plt.plot(real_time_list,error[j][0][:],colors_est[j])
    plt.xlabel('Time [sec]')
    plt.legend(["X [m]","Y [m]","Phi [rad]"])
    # plt.legend(["Estimated X Position [m]","Estimated Y Position [m]","Estimated Orientation [rad]"])
    plt.ylabel("Estimation Error")
    plt.grid()
    plt.show()

    #Control error plot
    plt.figure()
    x_d = [[3.0], [3.0], [0.0]]
    # x_d=np.array(rospy.get_param("x_d",x_d_init))
    for j in range(3):
        plt.plot(real_time_list,np.array(x_real_list[j][0][:])-x_d[j][:],colors_real[j])
        plt.plot(est_time_list,np.array(x_est_list[j][0][:])-x_d[j][:],colors_est[j])
    plt.xlabel('Time [sec]')
    plt.legend(["X_real [m]","X_est [m]","Y_real [m]", "Y_est [m]","Phi_real [rad]","Phi_est [rad]"])
    # plt.legend(["Estimated X Position [m]","Estimated Y Position [m]","Estimated Orientation [rad]"])
    plt.ylabel("Control Error")
    plt.grid()
    plt.show()