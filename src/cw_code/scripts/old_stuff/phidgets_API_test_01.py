#!/usr/bin/env python
import sys
import time
import traceback
import pprint


from Phidget22.Devices.Accelerometer import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *


import numpy as np
import scipy.integrate as integrate
import matplotlib.pyplot as plt


# Task: 
#   1) plot acceleration value from a single axis
#       1a) plot velocity and displacement values of that same axis on the same graph (using integration/ average function)


## Odometry class using phidgets IMU API ##



def make_acceleromter_obj():
    tmp = Accelerometer()
    tmp.setDeviceSerialNumber(302056)    # (-1 => first relevant and present element detected will be used).
    tmp.setHubPort(-1)
    tmp.setChannel(-1) 
    tmp.openWaitForAttachment(500)          # Timeout.
    tmp.getAttached()

    return(tmp)



# not an efficient method, would be better to use ISR to push sampled value onto buffer.
def sample_single_axis(IMU_obj, axis, n, t):

    x_y_z = dict(x=0,y=1,z=2)[axis]
    data_points = np.zeros(n)


    for i in range(n):

        data_points[i] = IMU_obj.getAcceleration()[x_y_z]   # Sample data.
        time.sleep(t)                                       # Wait before gathering next sample(s).

    return data_points




def setup_graph(input_data, n, t):
    x_vals = np.linspace(0,n*t,n)   #0-to-5 seconds, 10 samples.
    y_vals = input_data

    plt.plot(x_vals,y_vals,'o')
    plt.ylabel('acceleration (m/s^2)')
    plt.xlabel('time (s)')
    plt.show()
    time.sleep(t/10)
    plt.close()

if __name__ == '__main__':
    
    myIMU = make_acceleromter_obj()
    

    # Gather samples (very inneficiently!) #

    num_samples = 1             
    time_between_samples = 0.1  # Measure samples at a ~rate of 1-sample/ 100ms.

    my_samples = sample_single_axis(myIMU, 'x', num_samples, time_between_samples)    # Gather axial-accelerometer samples from phidget, at a rate of 1 sample/sec.

    # Create graph #
    
    setup_graph(my_samples, num_samples, time_between_samples)

    



    # while(1):

    #     x = myIMU.getAcceleration()[0]
    #     y = myIMU.getAcceleration()[1]
    #     z = myIMU.getAcceleration()[2]
    #     print(myIMU.getAcceleration())

    #     time.sleep(1.0)
    