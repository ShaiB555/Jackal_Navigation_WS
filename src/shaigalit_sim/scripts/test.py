import numpy as np
# #import scipy as sc
# import matplotlib.pyplot as plt

# #Sensor angle alpha
# N=720
# max_ang=2.3561899662017822
# alpha=np.linspace(-max_ang,max_ang,N)
# ranges=np.linspace(0,5,N)

# x_data=np.array([])
# y_data=np.array([])

# for i in range(N):
#     x_data=np.append(x_data,ranges[i]*np.cos(alpha[i]))
#     y_data=np.append(y_data,ranges[i]*np.sin(alpha[i]))

# plt.plot(x_data,y_data)
# plt.show()

# popt=[1,2,3]
# print(np.size(popt))
# # [x0,y0,R]=popt
# # print(y0)
# y0=1
# R=5
# x=1
# x0=3
# y0-np.sqrt(R**2-(x-x0)**2)
#%%
import numpy as np

N=4
print(np.diag(np.ones((2*N))))
print("atan")
print(np.arctan(1)/np.pi)

#%% Location in robot CS
xr= 1.247246
yr= 0.468049
phi= -0.285723

d=np.transpose(np.array([xr,yr,0]))
c_p=np.cos(phi)
s_p=np.sin(phi)
R=np.array([[c_p, s_p, 0],[-s_p, c_p, 0],[0, 0, 1]])
A=np.array([[c_p, -s_p, 0, xr],[s_p, c_p, 0, yr],[0, 0, 1, 0],[0,0,0,1]])
Ainv=np.linalg.inv(A)

#Beacon 1
xb=4
yb=2
pos=np.dot(Ainv,np.transpose(np.array([xb,yb,0,1])))
# Rt=np.transpose(R)
# pos=np.dot(R,np.transpose(np.array([xb-xr,yb-yr,0])))
print("Beacon 1")
print(pos)

#Beacon 2
xb=3.177278
yb=-0.972763
pos=np.dot(Ainv,np.transpose(np.array([xb,yb,0,1])))
# Rt=np.transpose(R)
# pos=np.dot(R,np.transpose(np.array([xb-xr,yb-yr,0])))
print("Beacon 2")
print(pos)

#Beacon 3
xb=4
yb=-2
pos=np.dot(Ainv,np.transpose(np.array([xb,yb,0,1])))
# Rt=np.transpose(R)
# pos=np.dot(R,np.transpose(np.array([xb-xr,yb-yr,0])))
print("Beacon 3")
print(pos)

#%%
import rospy
x_est_init=[[0],[0],[0]]
sigma_init=[[0.1,0,0],[0,0.1,0],[0,0,0.1]]
x_est=rospy.get_param("x_est",x_est_init)
sigma=rospy.get_param("sigma",sigma_init)
print(x_est)


# %%
