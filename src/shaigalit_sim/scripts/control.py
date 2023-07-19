#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import time
import angle_diff
import sys_mat

def control_callback(cont_law):

    rospy.init_node('jackal_controller')
    
    #PID Values - Position [Kp,Ki,Kd]
    K_pos=np.array([[3],[0],[0]])
    #PID Values - Path Orientation [Kp,Ki,Kd]
    K_ori=np.array([[3],[0],[0.05]])
    #PID Values - End Orientation [Kp,Ki,Kd]
    K_end=np.array([[1],[0.0],[0.1]])

    #LQR parameters
    Q_lqr=10*np.diag([1,1,np.pi/20])#*abs(error_dist)
    R_lqr=1*np.diag([1,np.pi/20])
    maxiter=150
    eps=0.001
    
    #Initial values
    x_d_init = [[0.0], [0.0], [0.0]]
    error_init = [[0.0], [0.0], [0.0]]
    x_est_init = [[0.0],[0.0],[0.0]]
    error_int_init = [[0.0], [0.0], [0.0]]
    x_d=np.array(rospy.get_param("x_d",x_d_init))
    x_est=np.array(rospy.get_param("x_est",x_est_init))
    max_v=2
    max_w=4
    # Phase_init=True

    #Calculating the time increment from the last iteration to the current time, dt:
    current_time = time.time()
    last_time=rospy.get_param("cont_last_time",0)
    dt = current_time - last_time
    rospy.set_param("cont_last_time",current_time)

    # Errors calculation
    error_prev = rospy.get_param("error",error_init)
    error_int = rospy.get_param("error_int",error_int_init)
    c_p=np.cos(x_est[2,0])
    s_p=np.sin(x_est[2,0])
    Rot=np.array([[c_p, s_p, 0],[-s_p, c_p, 0],[0, 0, 1]])
    error_x=Rot@(x_d-x_est)
    if abs(error_x[0,0])<0.001:
        theta=(np.sign(error_x[1,0]*error_x[0,0]))*np.pi/2
    else:
        theta=np.arctan(error_x[1,0]/error_x[0,0])
    phi_est=x_est[2,0]
    phi_d=x_d[2,0]
    error_ori_path=theta#angle_diff.angle_diff(theta,phi_est)#theta-phi_est
    error_ori_end=angle_diff.angle_diff(phi_d,phi_est)#phi_d-phi_est
    # if -np.pi/2<error_ori_path<np.pi/2: sign=1
    # else: sign=-1
    sign=np.sign(error_x[0,0])
    error_dist=np.sqrt(error_x[0,0]**2+error_x[1,0]**2)*sign
    error=[[float(error_dist)],[float(error_ori_path)],[float(error_ori_end)]]
    rospy.set_param("error",error)
    error_der=(np.array(error)-np.array(error_prev))/dt

    # Errors calculation
    error=np.array(error)
    error_int=np.array(error_int)
    error_prev=np.array(error_prev)
    error=np.array(error)
    error_der=np.array(error_der)

    #PID Control Law

    if cont_law=="PID":

        if abs(error_dist)>0.1: #path to goal
            rospy.set_param("at_goal",False)
            # if abs(error_ori_path)<0.05:
            v = K_pos[0,0]*error[0,0]+K_pos[1,0]*error_int[0,0]+K_pos[2,0]*error_der[0,0]
            w = K_ori[0,0]*error[1,0] +K_ori[1,0]*error_int[1,0]+K_ori[2,0]*error_der[1,0]#-0.2*beta
            if abs(error_dist)<0.2:
                w=K_end[0,0]*error[2,0]+K_end[1,0]*error_int[2,0]+K_end[2,0]*error_der[2,0]
            # else:
            #     v=0.1*(K_pos[0,0]*error[0,0]+K_pos[1,0]*error_int[0,0]+K_pos[2,0]*error_der[0,0])
            #     w=K_ori[0,0]*error[1,0] +K_ori[1,0]*error_int[1,0]+K_ori[2,0]*error_der[1,0]#-0.2*beta
        elif abs(error_ori_end)>0.1: #fixing orientation
            v = 0
            w = K_end[0,0]*error[2,0]+K_end[1,0]*error_int[2,0]+K_end[2,0]*error_der[2,0]
            rospy.set_param("at_goal",False)
        else: #stop moving!!!
            v=0
            w=0
            rospy.set_param("at_goal",True)
            print("DESTINATION REACHED! TIME TO PARTY :)")

    #LQR Control Law

    
    if cont_law=="LQR":
            
        u=rospy.get_param("u",[[0],[0]])
        error_x_world=x_d-x_est
        Ad,Bd=sys_mat.get_mat(x_est,u,dt)

        # first, try to solve the ricatti equation
        P = Q_lqr
        for i in range(maxiter):
            Pn = np.transpose(Ad) @ P @ Ad - np.transpose(Ad) @ P @ Bd @ np.linalg.inv(R_lqr + np.transpose(Bd) @ P @ Bd) @ np.transpose(Bd) @ P @ Ad + Q_lqr
            if (abs(Pn - P)).max() < eps:
                break
            P = Pn
        # compute the LQR gain
        K = np.linalg.inv(Bd.T @ P @ Bd + R_lqr) @ (Bd.T @ P @ Ad)
        # compute the LQR control effort
        u = K @ np.array(error_x_world)
        v=u[0,0]
        w=u[1,0]

        if abs(error_dist)<0.05 and abs(error_ori_end)<0.1:
            v=0
            w=0
            rospy.set_param("at_goal",True)
            print("DESTINATION REACHED! TIME TO PARTY :)")

    # if cont_law=="MPC":

    #Setting and saving the control inputs
    if abs(v)>max_v: v=np.sign(v)*max_v
    if abs(w)>max_w: w=np.sign(w)*max_w
    u=[[float(v)],[float(w)]]
    rospy.set_param("u",u)
    u=np.array(u)

    # Publish Twist message with control inputs
    cmd_vel_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    twist_msg = Twist()
    twist_msg.linear.x = u[0,0]
    twist_msg.angular.z = u[1,0]
    cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        cont_law="LQR"
        time.sleep(3)
        rospy.init_node('jackal_controller')
        at_goal=False
        rospy.set_param("at_goal",at_goal)
        rate = rospy.Rate(100)  # Adjust the rate as per your requirement
        
        # while not at_goal:
        counter=0
        while not rospy.is_shutdown():
            counter+=1
            controller = control_callback(cont_law)
            rate.sleep()
            at_goal = rospy.get_param("at_goal")
            # print(at_goal)
            if at_goal==True and counter>100: break

    except rospy.ROSInterruptException:
        pass