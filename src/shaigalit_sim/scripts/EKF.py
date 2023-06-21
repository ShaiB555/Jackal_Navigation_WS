#!/usr/bin/env python3
import numpy as np

#NEED TO VERIFY WHAT ARE THE CONTROLLED PARAMETERS!!!! RIGHT WE ASSUME U IS A 3X1 IN THE CODE, BUT PROBABLY 2X1 (CONTROLLING LINEAR VELOCITY OF X AND ANGULAR VELOCITY OF PHI)
#NEED TO CREATE NEW "MAIN" FUNCTION THAT CALLS LASER_SUB AND EKF AND PROBABLY THE CONTROLLER LATER --> LAUNCH FILE?

#EXTENDED KALMAN FILTER
def EKF(x_est,sigma,u,beacons_rel,beacons_pos,dt):
    mu_bar=predict(x_est,u,dt)
    Rt=0.1*np.diag(np.array([1,1,0.1*np.pi/6]))
    Gt=G_calc(x_est,u,dt) #SHOULD THE INPUT BE mu_bar OR x_est????
    sigma_bar=Gt@(sigma@(np.transpose(Gt)))+Rt
    Ht,Qt=H_beacons(x_est,beacons_pos)
    Kt=sigma_bar@((np.transpose(Ht))@np.linalg.inv(Ht@sigma_bar@np.transpose(Ht)+Qt))
    Z_bar=h_beacons(x_est,beacons_pos)
    dZ=beacons_rel-Z_bar
    x_est=mu_bar+Kt@(dZ)
    sigma = (np.diag([1,1,1])-Kt@Ht)@sigma_bar
    return x_est, sigma

#CALCULATING THE ESTIMATED POSITION OF A BEACON IN THE ROBOT FRAME FOR EKF, BASED ON THE KNOWN ROBOT POSE AND BEACON LOCATION IN THE WORLD FRAME
def h_calc(beacon_pos,x_est):
    xb=beacon_pos[0,0]
    yb=beacon_pos[1,0]
    x=x_est[0,0]
    y=x_est[1,0]
    phi=x_est[2,0]
    dx=xb-x
    dy=yb-y
    c_p=np.cos(phi)
    s_p=np.sin(phi)
    h=np.array([[dx*c_p+dy*s_p],[-dx*s_p+dy*c_p]])
    return h

#STACKING THE ESTIMATED POSITION OF ALL VISIBLE BEACONS, Z_bar, FOR USE IN EKF
def h_beacons(x_est,beacons_pos):
    Z_bar=np.array([0]) #ALLOWS TO USE VSTACK
    N=int(len(beacons_pos)/2) #N IS THE NUMBER OF BEACONS
    for i in range(N): #CALCULATES JACOBIAN FOR EACH BEACON, BUT ONLY IF IT'S VISIBLE
        beacon_pos=np.array([[beacons_pos[2*i,0]],[beacons_pos[2*i+1,0]]])
        temp=h_calc(beacon_pos,x_est)
        Z_bar=np.vstack((Z_bar,temp)) #STACKS THE JACOBIAN FOR EACH VISIBLE BEACON
    Z_bar=np.delete(Z_bar, 0, axis=0) #REMOVING THE LINE OF ZEROS
    return Z_bar

#CALCULATING THE JACOBIAN MATRIX OF THE MEASUREMENT OF ONE BEACON FOR EKF
#BASED ON THE X DERIVATIVE OF THE h(X) FUNCTION FOR ONE BEACON FOR X=[x,y,phi]
def H_calc(beacon_pos,x_est):
    xb=beacon_pos[0,0]
    yb=beacon_pos[1,0]
    x=x_est[0,0]
    y=x_est[1,0]
    phi=x_est[2,0]
    c_p=np.cos(phi)
    s_p=np.sin(phi)
    dx=xb-x
    dy=yb-y
    H=np.array([[-c_p,-s_p, -dx*s_p+dy*c_p],[s_p, -c_p, -dx*c_p-dy*s_p]])
    return H

#THIS IS THE OVERALL JACOBIAN MATRIX FOR ALL VISIBLE BEACONS, USED IN THE EKF
def H_beacons(x_est,beacons_pos):
    Ht=np.array([[0,0,0]]) #ALLOWS TO USE VSTACK
    N=int(len(beacons_pos)/2) #N IS THE NUMBER OF BEACONS
    Qt=0.1*np.diag(np.ones((2*N))) #THE Qt MATRIX REPRESENTS THE ACCURACY OF THE SENSOR, IN THIS CASE WE HAVE OBSERVED AN ERROR OF 0.1M IN THE LOCATION OF A BEACON
    for i in range(N): #CALCULATES JACOBIAN FOR EACH BEACON
        beacon_pos=np.array([[beacons_pos[2*i,0]],[beacons_pos[2*i+1,0]]])
        temp=H_calc(beacon_pos,x_est)
        Ht=np.vstack((Ht,temp)) #STACKS THE JACOBIAN FOR EACH VISIBLE BEACON
        # Qt[2*i,2*i]=0.5*Qt[2*i,2*i]
    Ht=np.delete(Ht, 0, axis=0) #REMOVING THE LINE OF ZEROS
    return Ht, Qt


#DYNAMICS OF THE SYSTEM FOR A GIVEN POSITON X AND CONTROL SIGNAL U
#JACKAL POSE IN THE WORLD FRAME: X=[x,y,phi]
#CONTROLLED PARAMETERS: U=[v,omega]
#X_dot=[x_dot,y_dot,phi_dot]=[v*cos(phi),v*sin(phi),omega]
#NEED TO LINEARIZE PLANT AROUND CURRENT POINT X
def predict(x,u,dt):
    v=u[0,0]
    w=u[1,0]
    phi=x[2,0]
    x=x+np.array([[v*np.cos(phi)],[v*np.sin(phi)],[w]])*dt
    return x

#JACOBIAN OF THE ROBOT'S DYNAMIC SYSTEM
def G_calc(x_est,u,dt):
    v=u[1,0]
    phi=x_est[2,0]
    Gt=np.array([[1,0,-v*dt*np.sin(phi)],[0,1,v*dt*np.cos(phi)],[0,0,1]])
    return Gt