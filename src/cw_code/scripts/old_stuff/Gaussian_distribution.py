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

topic_of_interest = "/imu/data_raw"

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
    linear_a_x.append(abs(data.linear_acceleration.x))
    linear_a_y.append(abs(data.linear_acceleration.y))
    linear_a_z.append(abs(data.linear_acceleration.z))

    #pprint.pprint(data)





#-- Subscriber "sniffs" topic of interest --#
def sniffer():
    rospy.init_node("Sniffer")
    rospy.Subscriber(topic_of_interest, Imu, append_to_list)





#-- Wipe array data --#
def format_arrays():
    del linear_a_x[:]
    del linear_a_y[:] 
    del linear_a_z[:] 

    del angular_a_x[:] 
    del angular_a_y[:] 
    del angular_a_z[:] 




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
    print("\nExiting...")
    if len(linear_a_x ) > 100:
        make_gaussian_distribution(linear_a_y)
    else:
        print('Take more samples, fewer than 100 were detected.\n\r')
    #pprint.pprint(slice_list(linear_a_x,10))






#-- Gaussing plot generator --#
def make_gaussian_distribution(data):

    #-- Sort magnitude data --#
    data = sorted(data)

    #-- Normalized data --#
    data_norm = []
    for i in data:
        data_norm.append((i - min(data))/(max(data) - min(data)))


    #-- Gather statistical info from data --#
    mu = np.mean(data)
    sigma = np.std(data)
    mode = stats.mode(data) # only global mode is returned, in this case.

                            #BUG: review histograms! (https://bespokeblog.wordpress.com/2011/07/11/basic-data-plotting-with-matplotlib-part-3-histograms/)
    
    #-- Setup Histogram parameters --#
    n, bins, patches = plt.hist(data, len(data)/20, normed=True, facecolor='green', alpha=0.75)

    #NOTE: bins => how many rectangles are used to represent data.
    #NOTE: Assuming noise has a normal distributoin. (is it more like a uniform distribution? :-o)

    #-- Adding 'best-fit' line --#
    y = mlab.normpdf(bins,mu,sigma)
    l = plt.plot(bins, y, 'r--', linewidth=2)

    #-- Adding graph labels (Readings vs Probabiblity) and title
    plt.xlabel('Reading [g]')
    plt.ylabel('Count')
    plt.title('IMU noise distribution: ' + r'$\mu=$' + ' {}, '.format(round(mu,3)) + r'$\sigma=$'+ '{}'.format(round(sigma,6)))
   
    #pprint.pprint(data_norm)

    #-- Adding remaining graph features --#
    plt.axis([min(data)-2*sigma, max(data)+2*sigma, 0, 200])        #NOTE: need to calculate count mode-range (not just one value) => use bins
    plt.grid(True)

    plt.show()
    #plt.show(block=False)










def main():
    try:
    
        #-- Setup Listener --#
        sniffer()         
        #-- React to node kill --#
        rospy.on_shutdown(shutdown_handler)     

        #-- Spin --#
        while not rospy.is_shutdown():
            #rospy.spin()

            if (len(linear_a_x) >= 2000):

                #-- plot data --#
                make_gaussian_distribution(linear_a_y)

                #-- Format arrays for reuse --#
                format_arrays()

            #-- idle CPU until a message is detected --#
            rospy.wait_for_message(topic_of_interest, Imu)  #FANS WILL BE VERY LOUD IF THIS ISN'T USED!
            
        return 0

    #-- React to keyboard Interrupt --#
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt pressed.")
        shutdown_handler()
        return 0

if __name__ == '__main__':
    main()

