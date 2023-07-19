#!/usr/bin/env python3
import numpy as np

def get_mat(x_est,u,dt):
    u=np.array(u)
    v=u[0,0]
    w=u[1,0]
    x=x_est[0,0]
    y=x_est[1,0]
    phi=x_est[2,0]
    A=np.array([[0,0,-v*np.sin(phi)],[0,0,v*np.cos(phi)],[0,0,0]])
    B=np.array([[np.cos(phi),0],[np.sin(phi),0],[0,1]])
    Ad=A*dt+np.eye(3)
    Bd=B*dt
    return Ad,Bd