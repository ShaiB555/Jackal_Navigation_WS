import numpy as np

# # Example array
# arr = np.array([[1, 2, 3],
#                 [4, 5, 6],
#                 [7, 8, 9]])

# # Remove the second horizontal line (index 1)
# new_arr = np.delete(arr, 1, axis=0)

# # Print the updated array
# print(new_arr)

#PID Values - Position
Kp = 1.0  # Proportional gain
Ki = 0.0  # Integral gain
Kd = 0.0  # Derivative gain
K_pos=[Kp,Ki,Kd]
#PID Values - Orientation
Kp = 2.0  # Proportional gain
Ki = 0.0  # Integral gain
Kd = 0.0  # Derivative gain
K_ori=[Kp,Ki,Kd]
print(K_pos)
print(K_ori)