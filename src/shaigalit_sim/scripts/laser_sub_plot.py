#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from scipy.optimize import curve_fit
import circle_find
import EKF
import time


def circ(pos,x0,y0,R):
    return (pos[0]-x0)**2+(pos[1]-y0)**2-R**2    


def callback(data):
       
    #Real beacon positions and radius
    b1=np.array([0,0,0.15])
    b2=np.array([6,6,0.25])
    b3=np.array([-6,-6,0.31])
    b4=np.array([-6,6,0.46])
    b5=np.array([6,-6,0.54])
    beacons_real=np.vstack((b1,b2,b3,b4,b5))
    print(beacons_real)
    
    #USING SENSOR DATA TO FIND THE POSITION OF THE VISIBLE BEACONS IN THE ROBOT FRAME
    ranges=data.ranges
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    
    #Sensor angle alpha
    N=np.size(ranges)
    alpha=np.linspace(angle_min,angle_max,N)

    #Initialize
    x_data=np.array([])
    y_data=np.array([])
    results=np.array([0,0,0])
    continuous=0.1
    start=False

    #Gathering points on an object and fitting to circle function for each object
    for i in range(N):
        if ranges[i] != np.inf and ranges[i] != np.NaN:
            if i>0:
                if abs(ranges[i]-ranges[i-1])<continuous:
                    start=True
                    x_data=np.append(x_data,ranges[i]*np.cos(alpha[i]))
                    y_data=np.append(y_data,ranges[i]*np.sin(alpha[i]))
                else:
                    start=False
                    #fit to circle curve
                    try:
                        popt, _ = curve_fit(circ, (x_data, y_data),np.zeros_like(x_data))
                        popt[2]=abs(popt[2])
                        if popt[2]<1: #filtering out objects with radius larger than 1m (largest beacon is 0.54m radius)
                            results=np.vstack((results,popt))
                    except ValueError:
                        pass
                    except TypeError:
                        pass
                    except RuntimeError:
                        pass
                    x_data=np.array([])
                    y_data=np.array([])
            else:
                start=True
                x_data=np.append(x_data,ranges[i]*np.cos(alpha[i]))
                y_data=np.append(y_data,ranges[i]*np.sin(alpha[i]))
        elif start==True:
            start=False
            #fit to circle curve
            try:
                popt, _ = curve_fit(circ, (x_data, y_data),np.zeros_like(x_data))
                popt[2]=abs(popt[2])
                if popt[2]<1:
                    results=np.vstack((results,popt))
            except ValueError:
                pass
            except TypeError:
                pass
            except RuntimeError:
                pass
            x_data=np.array([])
            y_data=np.array([])
    
    

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("front/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    # rospy.shutdown()  # Shut down ROS after processing the data

if __name__ == '__main__':
    listener()