import rospy
import numpy as np
from matplotlib import pyplot as plt

def remove_first_column(matrix):
    return [row[1:] for row in matrix]

if __name__ == '__main__':

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
   
    x_est_list=[[est_x_list],[est_y_list],[est_yaw_list]]
    
    est_time_list=est_time_list[1:]
    est_time_list=np.array(est_time_list)-est_time_list[0]
    real_time_list=real_time_list[1:]
    real_time_list=np.array(real_time_list)-real_time_list[0]
    plt.figure()
    colors_est=["r-","g-","b-"]
    colors_real=["r--","g--","b--"]
    # print("REAL")
    # print(np.size(x_real_vec))
    # print("EST")
    # print(np.size(x_est_vec)) /// [:len(x_real_vec[i])]
    for i in range(3):
        plt.plot(real_time_list[:len(x_real_list[i])],x_real_list[i],colors_real[i])
        plt.plot(est_time_list[:len(x_est_list[i])],x_est_list[i],colors_est[i])
    plt.xlabel('Time [sec]')
    # plt.legend(["Estimated X Position [m]","Estimated Y Position [m]","Estimated Orientation [rad]"])
    plt.grid()
    plt.show()

    #World map + XY motion
    