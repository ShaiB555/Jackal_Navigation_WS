#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import LaserScan
from scipy.optimize import curve_fit


def circ(pos,x0,y0,R):
    return (pos[0]-x0)**2+(pos[1]-y0)**2-R**2

def R_sqaured(popt,pcov,x_data, y_data):

    # Calculate the residuals (difference between data and fitted values)
    residuals = circ((x_data, y_data), popt[0], popt[1], popt[2])

    # Calculate the total sum of squares (TSS)
    tss = np.sum((x_data - np.mean(x_data))**2 + (y_data - np.mean(y_data))**2)

    # Calculate the sum of squared residuals (RSS) from the circle fit
    rss = np.sum(residuals**2)

    # Calculate the R-squared value
    r_squared = 1 - (rss / tss)

    return r_squared


if __name__ == '__main__':
    # listener()

    rospy.init_node('listener', anonymous=True)

    data = rospy.wait_for_message("front/scan", LaserScan)
        
    #Real beacon positions and radius
    b1=np.array([0,0,0.15])
    b2=np.array([6,6,0.25])
    b3=np.array([-6,-6,0.31])
    b4=np.array([-6,6,0.46])
    b5=np.array([6,-6,0.54])
    beacons_real=np.vstack((b1,b2,b3,b4,b5))

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
    x_data_beacon=np.array([])
    y_data_beacon=np.array([])
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
                        popt, pcov = curve_fit(circ, (x_data, y_data),np.zeros_like(x_data))
                        popt[2]=abs(popt[2])
                        R2=R_sqaured(popt,pcov,x_data, y_data)
                        if popt[2]<1 and R2>0.999: #filtering out objects with radius larger than 1m (largest beacon is 0.54m radius)
                            results=np.vstack((results,popt))
                            x_data_beacon=x_data
                            y_data_beacon=y_data
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
                popt, pcov = curve_fit(circ, (x_data, y_data),np.zeros_like(x_data))
                popt[2]=abs(popt[2])
                R2=R_sqaured(popt,pcov,x_data, y_data)
                if popt[2]<1 and R2>0.999:
                    results=np.vstack((results,popt))
                    x_data_beacon=x_data
                    y_data_beacon=y_data
            except ValueError:
                pass
            except TypeError:
                pass
            except RuntimeError:
                pass
            x_data=np.array([])
            y_data=np.array([])

    #Plotting the graph of the circle and measurements
    plt.figure()

    plt.scatter(x_data_beacon,y_data_beacon,color='red')

    x0=results[-1,0]
    y0=results[-1,1]
    R=results[-1,2]

    theta = np.linspace(0, 2*np.pi, 100)
    x = x0 + R * np.cos(theta)
    y = y0 + R * np.sin(theta)
    
    
    plt.plot(x, y)
    plt.axis('equal')
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.grid()

    

    vec=np.array([[b4[0]+3.996725],[b4[1]+2.9890033],[0]])
    phi=0.302783
    s_p=np.sin(phi)
    c_p=np.cos(phi)
    Rot=np.array([[c_p, s_p, 0],[-s_p, c_p, 0],[0, 0, 1]])

    vec=Rot@vec-np.array([[0.12],[0],[0]])

    x4 = vec[0] + b4[2]* np.cos(theta)
    y4= vec[1] + b4[2]* np.sin(theta)

    plt.plot(x4,y4)

    errors=vec-np.array([[x0],[y0],[0]])
    print(errors)

    plt.text(-0.2,9.5, f'Center Position Error:\nX = {np.round(errors[0,0],5)}'+"[m]"+f'\nY ={np.round(errors[1,0],5)} [m]', fontsize = 10,bbox = dict(facecolor = 'white', alpha = 0.5))

    plt.legend(["Curve Fitting","Real Beacon","Measurements"])
    plt.title("Beacon Real and Fitted Position in the Robot Coordinate System")
    plt.show()