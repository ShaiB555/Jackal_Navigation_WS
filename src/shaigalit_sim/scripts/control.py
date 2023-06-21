#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from math import atan2, sin, cos, sqrt
import time
import angle_diff

def control_callback(K_pos,K_ori):

    rospy.init_node('jackal_controller')
    
    #Initial values
    x_d_init = [[0.0], [0.0], [0.0]]
    error_init = [[0.0], [0.0], [0.0]]
    x_est_init = [[0.0],[0.0],[0.0]]
    error_int_init = [[0.0], [0.0], [0.0]]
    x_d=np.array(rospy.get_param("x_d",x_d_init))
    x_est=np.array(rospy.get_param("x_est",x_est_init))
    max_v=10
    max_w=2
    Phase_init=True

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
    print(error_ori_path)
    error_ori_end=angle_diff.angle_diff(phi_d,phi_est)#phi_d-phi_est
    # if -np.pi/2<error_ori_path<np.pi/2: sign=1
    # else: sign=-1
    # sign=np.sign(np.cos(error_ori_path))
    # print("SIGN")
    # print(sign)
    error_dist=np.sqrt(error_x[0,0]**2+error_x[1,0]**2)
    error=[[float(error_dist)],[float(error_ori_path)],[float(error_ori_end)]]
    rospy.set_param("error",error)
    error_der=(np.array(error)-np.array(error_prev))/dt

    # PID controller
    error=np.array(error)
    print("ERROR")
    print(error)
    error_int=np.array(error_int)
    error_prev=np.array(error_prev)
    error=np.array(error)
    error_der=np.array(error_der)
    # print("ERROR INT")
    # print(error_int)
    Phase1=rospy.get_param("Phase1",Phase_init)
    Phase2=rospy.get_param("Phase2",Phase_init)
    Phase3=rospy.get_param("Phase3",Phase_init)
    print("PHASE 1")
    print(Phase1)
    print("PHASE 2")
    print(Phase2)

    #USING THE METHOD FROM PYHTONROBOTICS MOVE_TO_POSE
    alpha=angle_diff.angle_diff(theta,phi_est)#(theta-phi_est)%(2*np.pi)-np.pi
    beta=angle_diff.angle_diff(phi_d,phi_est)#(phi_d-phi_est)%(2*np.pi)-np.pi
    rho=np.hypot(error_x[1,0],error_x[0,0])
    # rho_prev=error_prev[0,0]
    # rho_der=(rho-rho_prev)/dt
    if abs(rho)>0.1:
        v = K_pos[0,0]*rho
        w = K_ori[0,0]*(alpha-0.2*beta)
    elif beta>0.05:
        v = 0
        w = 0.5*K_ori[0,0]*beta
    else:
        v=0
        w=0
        rospy.set_param("at_goal",True)
        print("DESTINATION REACHED!")
                                
    # if abs(error[1,0])>0.05 and Phase1==True:
    #     #Initial orientation to goal position:
    #     error_int[0,0]=error[0,0]+error_prev[0,0]
    #     v=0
    #     w=K_ori[0,0]*error[1,0]+K_ori[1,0]*error_int[1,0]+K_ori[2,0]*error_der[1,0]
    #     error_int=error_int.tolist()
    #     rospy.set_param("error_int",error_int)
    # elif abs(error[0,0])>0.3 and Phase2==True:
    #     #Phase 2: distance control:
    #     Phase1=False
    #     rospy.set_param("Phase1",Phase1)
    #     #Closing the distance to goal position:
    #     error_int[1,0]=error[1,0]+error_prev[1,0]
    #     v=K_pos[0,0]*error[0,0]+K_pos[1,0]*error_int[0,0]+K_pos[2,0]*error_der[0,0]
    #     w=K_ori[0,0]*error[1,0]+K_ori[1,0]*error_int[1,0]+K_ori[2,0]*error_der[1,0]
    #     w=0.1*w
    #     error_int=error_int.tolist()
    #     rospy.set_param("error_int",error_int)
    # elif abs(error[2,0])>0.1 and Phase3==True:
    #     Phase2=False
    #     rospy.set_param("Phase2",Phase2)
    #     #Correcting to desired orientation:
    #     error_int[2,0]=error[2,0]+error_prev[2,0]
    #     v=0
    #     w=K_ori[0,0]*error[2,0]+K_ori[1,0]*error_int[2,0]+K_ori[2,0]*error_der[2,0]
    #     error_int=error_int.tolist()
    #     rospy.set_param("error_int",error_int)
    # else:
    #     Phase1=False
    #     Phase2=False
    #     Phase3=False
    #     rospy.set_param("Phase3",Phase3)
    #     #Destination reached within desired error envelope
    #     v=0
    #     w=0

    #Setting and saving the control inputs
    if abs(v)>max_v: v=np.sign(v)*max_v
    if abs(w)>max_w: w=np.sign(w)*max_w
    u=[[float(v)],[float(w)]]
    rospy.set_param("u",u)
    u=np.array(u)
    print("U")
    print(u)

    # Publish Twist message with control inputs
    cmd_vel_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    twist_msg = Twist()
    twist_msg.linear.x = u[0,0]
    twist_msg.angular.z = u[1,0]
    cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        #PID Values - Position
        Kp = 0.5  # Proportional gain
        Ki = 0.0  # Integral gain
        Kd = 0.5  # Derivative gain
        K_pos=np.array([[Kp],[Ki],[Kd]])
        #PID Values - Orientation
        Kp = 5  # Proportional gain
        Ki = 0.0  # Integral gain
        Kd = 0.5  # Derivative gain
        K_ori=np.array([[Kp],[Ki],[Kd]])
        rospy.init_node('jackal_controller')
        at_goal=False
        rospy.set_param("at_goal",at_goal)
        rate = rospy.Rate(50)  # Adjust the rate as per your requirement
        while not at_goal:
            while not rospy.is_shutdown():
                controller = control_callback(K_pos, K_ori)
                rate.sleep()
                at_goal = rospy.get_param("at_goal")
                print(at_goal)
                # if at_goal==True: break
    except rospy.ROSInterruptException:
        pass
