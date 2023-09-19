import numpy as np

x=1
y=2
yaw=3
x_real_vec=[[0],[0],[0]]
x_real = [[x],[y], [yaw]]
x_real_vec=np.hstack((x_real_vec,x_real))
x_real_vec=x_real_vec.tolist()
print(x_real_vec)