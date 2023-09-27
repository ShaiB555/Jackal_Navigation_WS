import numpy as np

# x=1
# y=2
# yaw=3
# x_real_vec=[[0],[0],[0]]
# x_real = [[x],[y], [yaw]]
# x_real_vec=np.hstack((x_real_vec,x_real))
# x_real_vec=x_real_vec.tolist()
# print(x_real_vec)


def extract_first_elements(arr, m):
    if m <= 0:
        return []  # Return an empty list if m is not a positive integer

    return [arr[i] for i in range(0, len(arr), m)]

import numpy as np

def reduce_measurement_frequency(matrix, m):
    if m <= 0:
        return None  # Return None if m is not a positive integer

    num_rows, num_columns = matrix.shape
    num_reduced_columns = num_columns // m
    reduced_matrix = np.zeros((num_rows, num_reduced_columns))

    for i in range(num_reduced_columns):
        start_col = i * m
        end_col = (i + 1) * m
        reduced_col = matrix[:, start_col:end_col][:, 0]  # Take the first measurement from each group of m columns
        reduced_matrix[:, i] = reduced_col

    return reduced_matrix

# Example usage:
n = 100  # Number of time points
m = 10   # Reduce measurement frequency by taking the first measurement from every 10 columns

# Creating an example matrix with 3 measurements per time point
original_matrix = np.arange(1, n * 3 + 1).reshape(n, 3)

result_matrix = reduce_measurement_frequency(original_matrix, m)
print(result_matrix)


# Example usage:
n = 100
m = 10
original_array = list(range(1, n + 1))  # Creating an example array from 1 to 100
result_array = extract_first_elements(original_array, m)
# print(result_array)
