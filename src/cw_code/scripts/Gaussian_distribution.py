#!/usr/bin/env python
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from scipy import stats
import pprint

#-- SAMPLE IMU FOR A PREDEFINED DURATION (USE ROSBAG METADATA TO TRUNCATE EXCESS DATA IF NEEDED)
#-- SHOW GAUSSIAN DISTRIBUTION OF MEASURED VALUE (X-AXIS) AGAINST PROBABILITY (Y-AXIS)
#-- IN THIS CASE "PROBABILITY" REFERS TO THE AVERAGE OF THAT PARTICULAR VALUE (OR WINDOW OF VALUES).







import rospy 
from std_msgs.msg       import Float64
from geometry_msgs.msg  import Vector3
from sensor_msgs.msg    import Imu

topic_of_interest = "/imu/data"

#-- Allocating dynamic memory to each IMU sensor data-stream --#
linear_a_x = []
linear_a_y = []
linear_a_z = []

angular_a_x = []
angular_a_y = []
angular_a_z = []

#-- NOTE: Magnetometer data is only available in "/imu/data_raw" topic --#
# magnetometer_x = []
# magnetometer_y = []
# magnetometer_z = []


#-- Subscriber call-back pushing data to respective list --#
def append_to_list(data):
    linear_a_x.append(data.linear_acceleration.x)
    linear_a_y.append(data.linear_acceleration.y)
    linear_a_z.append(data.linear_acceleration.z)


#-- Subscriber "sniffs" topic of interest --#
def sniffer():
    rospy.init_node("Sniffer")
    rospy.Subscriber(topic_of_interest, Imu, append_to_list)



def slice_list(list, slices):


    tmp = []

    #-- Make sure list is in ascending order --#
    list = sorted(list)

    #-- Check if desired number of slices fits in list --#
    if len(list) > slices:

        #-- Number of slices that fit in the current list --#
        slice_width  = len(list)/slices

        for cut in range(slices):
            
            start   = cut*slice_width
            end     = (cut+1)*(slice_width-1)

            tmp.append(list[start : end])

        #return sliced lis --#
        return tmp

    else:
        print("Invalid list to slice ratio")
        return -1


#-- Run gaussian-plot generator from this handler --#
def shutdown_handler():
    print("Goodbye!!")
    make_gaussian_distribution(linear_a_x)
    #pprint.pprint(slice_list(linear_a_x,10))


#-- Gaussing plot generator --#
def make_gaussian_distribution(data):

    #-- Sort data --#
    data = sorted(data)

    #-- Gather statistical info from data --#
    mu = np.mean(data)
    sigma = np.std(data)
    mode = stats.mode(data) # only global mode is returned, in this case.

                            #BUG: review histograms! (https://bespokeblog.wordpress.com/2011/07/11/basic-data-plotting-with-matplotlib-part-3-histograms/)
    
    #-- Setup Histogram parameter --#
    n, bins, patches = plt.hist(data, 150, normed=1, facecolor='green', alpha=0.75)

    #NOTE: bins => how many rectangles are used to represent data.
    #NOTE: Assuming noise has a normal distributoin. (is it more like a uniform distribution? :-o)

    #-- Adding 'best-fit' line --#
    y = mlab.normpdf(bins,mu,sigma)

    #-- Adding graph labels (Readings vs Probabiblity) and title
    plt.xlabel('Reading [m/s^2]')
    plt.ylabel('Probability [%]')
    plt.title('IMU noise distribution')

    #-- Adding remaining graph features --#
    plt.axis([min(data), max(data), 0, 500])        #NOTE: need to calculate count mode-range (not just one value) => use bins


    plt.show()




def main():
    try:
    
        #-- Setup Listener --#
        sniffer()         
        #-- React to node kill --#
        rospy.on_shutdown(shutdown_handler)     

        #-- Spin --#
        while not rospy.is_shutdown():
            rospy.spin()

    #-- React to keyboard Interrupt --#
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt pressed.")
        shutdown_handler()

if __name__ == '__main__':
    main()

