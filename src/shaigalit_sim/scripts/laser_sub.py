#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from scipy.optimize import curve_fit
import circle_find
import EKF
import time


def callback(data):
    
    #KNOWN BEACON POSITIONS AND RADII
    beacon1_pos=np.array([[6],[6]])
    beacon2_pos=np.array([[-6],[-6]])
    beacon3_pos=np.array([[-6],[6]])
    beacon4_pos=np.array([[6],[-6]])
    # beacons_pos_all=np.array([[6],[6],[-6],[-6],[-6],[6],[6],[-6]])
    beacons_rad=np.array([0.25,0.31,0.46,0.54])
    
    #INITIAL GUESS FOR KALMAN FILTER
    x_est_init=[[0],[0],[0]]
    sigma_init=[[0.1,0,0],[0,0.1,0],[0,0,0.1]]
    u_init=[[0],[0]]

    #USING SENSOR DATA TO FIND THE POSITION OF THE VISIBLE BEACONS IN THE ROBOT FRAME
    ranges=data.ranges
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    results=circle_find.circles(ranges,angle_min,angle_max,angle_increment)
    beacons_visible=np.array([False,False,False,False])
    beacons_pos_vis=np.array([0]) #ENABLING VSTACK OF TRUE POSITION OF VISIBLE BEACONS
    error=0.01
    beacons_rel=np.array([0]) #ENABLING VSTACK OF RELATIVE POSITION OF VISIBLE BEACONS
    
    #Calculating distance from obstacles in front and back side
    collision_dist=100000000*[1,1]
    for i in range(len(ranges)):
        temp=ranges[i]
        alpha=angle_min+i*angle_increment
        if abs(alpha)<1:
            if temp<abs(collision_dist[0]):
                collision_dist[0]=-temp*np.sign(alpha)
        if abs(alpha)>np.pi-1:
            if temp<abs(collision_dist[1]):
                collision_dist[1]=temp*np.sign(alpha)
    rospy.set_param("collision_dist",collision_dist)

    #IDENTIFYING EACH VISIBLE BEACON BASED ON KNOWN RADIUS AND COLLECTING IN THE VECTOR beacons_rel
    for i in range(len(results)):
        x0=results[i,0]+0.12 #0.12m is the distance from the robot COM and the LIDAR
        y0=results[i,1]
        R=abs(results[i,2])

        if beacons_rad[0]-error<R<beacons_rad[0]+error:
            beacon1_rel=np.array([[x0],[y0]])
            # print(f"Beacon 1 of radius "+str(R)+" at ("+str(x0)+","+str(y0)+")")
            beacons_visible[0]=True

        if beacons_rad[1]-error<R<beacons_rad[1]+error:
            beacon2_rel=np.array([[x0],[y0]])
            # print(f"Beacon 2 of radius "+str(R)+" at ("+str(x0)+","+str(y0)+")")
            beacons_visible[1]=True

        if beacons_rad[2]-error<R<beacons_rad[2]+error:
            beacon3_rel=np.array([[x0],[y0]])
            # print(f"Beacon 3 of radius "+str(R)+" at ("+str(x0)+","+str(y0)+")")
            beacons_visible[2]=True

        if beacons_rad[3]-error<R<beacons_rad[3]+error:
            beacon4_rel=np.array([[x0],[y0]])
            # print(f"Beacon 4 of radius "+str(R)+" at ("+str(x0)+","+str(y0)+")")
            beacons_visible[3]=True           
        
    if beacons_visible[0]==True:
        beacons_rel=np.vstack((beacons_rel,beacon1_rel))
        beacons_pos_vis=np.vstack((beacons_pos_vis,beacon1_pos))

    if beacons_visible[1]==True:
        beacons_rel=np.vstack((beacons_rel,beacon2_rel))
        beacons_pos_vis=np.vstack((beacons_pos_vis,beacon2_pos))

    if beacons_visible[2]==True:
        beacons_rel=np.vstack((beacons_rel,beacon3_rel))
        beacons_pos_vis=np.vstack((beacons_pos_vis,beacon3_pos))

    if beacons_visible[3]==True:
        beacons_rel=np.vstack((beacons_rel,beacon4_rel))
        beacons_pos_vis=np.vstack((beacons_pos_vis,beacon4_pos))
    
    beacons_rel=np.delete(beacons_rel, 0, axis=0) #REMOVING THE ORIGINAL LINE OF ZEROS
    beacons_pos_vis=np.delete(beacons_pos_vis, 0, axis=0) #REMOVING THE ORIGINAL LINE OF ZEROS


    #CALCULATING THE DELTA TIME dt OF ACTIVE INPUT u FOR KALMAN FILTER
    current_time = time.time()
    last_time=rospy.get_param("last_time",0)
    dt = current_time - last_time
    rospy.set_param("last_time",current_time)

    #GETTING THE CONTROL SIGNAL u
    u=np.array(rospy.get_param("u",u_init))

    # print("BEFORE!")
    x_est=rospy.get_param("x_est",x_est_init)
    x_est=np.array(x_est)
    sigma=rospy.get_param("sigma",sigma_init)
    sigma=np.array(sigma)
    # print(x_est)
    x_est, sigma=EKF.EKF(x_est,sigma,u,beacons_rel,beacons_pos_vis,dt)
    # print("AFTER!")
    # print(x_est)
    x_est=x_est.tolist()
    rospy.set_param("x_est",x_est)
    sigma=sigma.tolist()
    rospy.set_param("sigma",sigma)
    

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("front/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()