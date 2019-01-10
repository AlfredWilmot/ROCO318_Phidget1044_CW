"""
=====
Decay
=====

This example showcases a sinusoidal decay animation.
"""


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time 
from scipy.interpolate import *




from Phidget22.Devices.Accelerometer import *

def make_acceleromter_obj():
    tmp = Accelerometer()
    tmp.setDeviceSerialNumber(302056)    # (-1 => first relevant and present element detected will be used).
    tmp.setHubPort(-1)
    tmp.setChannel(-1) 
    tmp.openWaitForAttachment(500)          # Timeout.
    tmp.getAttached()

    return(tmp)


myIMU = make_acceleromter_obj()




def single_integrate(x_prev, dv, dt):
    # x_n = x_(n-1) + v_n/dt        <=      next position is the previous position + current velocity divided by time-step.
   return x_prev + dv/dt


# Gather samples...
# not an efficient method, would be better to use ISR to push sampled value onto buffer.
def sample_single_axis(IMU_obj, axis, n, t):

    x_y_z = dict(x=0,y=1,z=2)[axis]
    data_points = np.zeros(n)


    for i in range(n):

        data_points[i] = IMU_obj.getAcceleration()[x_y_z]   # Sample data.
        time.sleep(t)                                       # Wait before gathering next sample(s).

    return data_points


num_samples = 1             
time_between_samples = 0.01  # Measure samples at a ~rate of 1-sample/ 100ms.






def data_gen(t=0):
    cnt = 0
    # while cnt < 1000:
    #     cnt += 1
    #     t += 0.1
    #     yield t, np.sin(2*np.pi*t) * np.exp(-t/10.)
    while cnt < 1000:
        cnt += 1
        t +=time_between_samples

        my_samples_x = sample_single_axis(myIMU, 'x', num_samples, time_between_samples) 
        my_samples_y = sample_single_axis(myIMU, 'y', num_samples, time_between_samples)

        yield my_samples_x, my_samples_y

def init():
    ax.set_ylim(-2, 2)
    ax.set_xlim(-2, 2)
    del xdata[:]
    del ydata[:]
    line.set_data(xdata, ydata)
    return line,

fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.grid()
xdata, ydata = [], []


def run(data):
    # update the data
    t, y = data
    xdata.append(t)
    ydata.append(y)
    xmin, xmax = ax.get_xlim()

    if t >= xmax:
        ax.set_xlim(xmin, 2*xmax)
        ax.figure.canvas.draw()
    line.set_data(xdata, ydata)

    return line,

ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10,
                              repeat=False, init_func=init)
plt.show()