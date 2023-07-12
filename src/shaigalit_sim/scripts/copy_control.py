#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import time
import angle_diff

def control_callback(K_pos,K_ori):

    rospy.init_node('jackal_controller')
    
    #Initial values
    x_d_init = [[0.0], [0.0], [0.0]]
    error_init = [[0.0], [0.0], [0.0]]
    x_est_init = [[0.0],[0.0],[0.0]]
    error_int_init = [[0.0], [0.0], [0.0]]
    x_d = x_d_init #rospy.get_param("x_d", x_d_init)
    x_est=np.array(rospy.get_param("x_est",x_est_init))
    max_v=10
    max_w=2

    #Calculating the time increment from the last iteration to the current time, dt:
    current_time = time.time()
    last_time=rospy.get_param("cont_last_time",0)
    dt = current_time - last_time
    rospy.set_param("cont_last_time",current_time)

    # Errors calculation
    error_prev = rospy.get_param("error",error_init)
    error_int = rospy.get_param("error_int",error_int_init)
    error_x=x_d-x_est
    if abs(error_x[0,0])<0.001:
        theta=(np.sign(error_x[1,0]*error_x[0,0]))*np.pi/2
    else:
        theta=np.arctan(error_x[1,0]/error_x[0,0])
    phi_est=x_est[2,0]
    phi_d=x_d[2,0]
    error_ori_path=angle_diff.angle_diff(theta,phi_est)#theta-phi_est
    error_ori_end=angle_diff.angle_diff(phi_d,phi_est)#phi_d-phi_est
    error_dist=np.sqrt(error_x[0,0]**2+error_x[1,0]**2)
    error=[[float(error_dist)],[float(error_ori_path)],[float(error_ori_end)]]
    rospy.set_param("error",error)
    error_der=(np.array(error)-np.array(error_prev))/dt

    # PID controller
    error=np.array(error)
    error_int=np.array(error_int)
    error_prev=np.array(error_prev)
    error=np.array(error)
    error_der=np.array(error_der)

    #USING THE METHOD FROM PYHTONROBOTICS MOVE_TO_POSE
    if abs(error_dist)>0.05: #path to goal
        v = K_pos[0,0]*error[0,0]+K_pos[1,0]*error_int[0,0]+K_pos[2,0]*error_der[0,0]
        w = K_ori[0,0]*error[1,0] +K_ori[1,0]*error_int[1,0]+K_ori[2,0]*error_der[1,0]#-0.2*beta
        rospy.set_param("at_goal",False)
    elif abs(error_ori_end)>0.05: #fixing orientation
        v = 0
        w = K_ori[0,0]*error[2,0]+K_ori[1,0]*error_int[2,0]+K_ori[2,0]*error_der[2,0]
        rospy.set_param("at_goal",False)
    else: #stop moving!!!
        v=0
        w=0
        rospy.set_param("at_goal",True)
        print("DESTINATION REACHED! TIME TO PARTY :)")

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

        #PID Values - Position
        Kp = 0.5 # Proportional gain
        Ki = 0.0  # Integral gain
        Kd = 0.0  # Derivative gain
        K_pos=np.array([[Kp],[Ki],[Kd]])
        #PID Values - Orientation
        Kp = 5  # Proportional gain
        Ki = 0.0  # Integral gain
        Kd = 0.1 # Derivative gain
        K_ori=np.array([[Kp],[Ki],[Kd]])
        rospy.init_node('jackal_controller')
        at_goal=False
        rospy.set_param("at_goal",at_goal)
        rate = rospy.Rate(50)  # Adjust the rate as per your requirement

        # while not at_goal:
        counter=0
        while not rospy.is_shutdown():
            counter+=1
            controller = control_callback(K_pos, K_ori)
            rate.sleep()
            at_goal = rospy.get_param("at_goal")
            if at_goal==True and counter>100: break
            
    except rospy.ROSInterruptException:
        pass
