#!/usr/bin/env python
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

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

def circles(ranges,angle_min,angle_max,angle_increment):
    #Sensor angle alpha
    N=np.size(ranges)
    alpha=np.linspace(angle_min,angle_max,N)

    #find points on a circle
    x_data=np.array([])
    y_data=np.array([])
    results=np.array([0,0,0])
    continuous=0.1
    start=False
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
                        R2=R_sqaured(popt,_,x_data, y_data)
                        if popt[2]<1 and R2>0.999: #filtering out objects with radius larger than 1m (largest beacon is 0.54m radius)
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
                R2=R_sqaured(popt,_,x_data, y_data)
                if popt[2]<1 and R2>0.999: #filtering out objects with radius larger than 1m (largest beacon is 0.54m radius)
                    results=np.vstack((results,popt))
            except ValueError:
                pass
            except TypeError:
                pass
            except RuntimeError:
                pass
            x_data=np.array([])
            y_data=np.array([])
    return results



























    


  


    