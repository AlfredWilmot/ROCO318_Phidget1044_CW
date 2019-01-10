#!/usr/bin/env python
import sys
import time
import traceback
import pprint


from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Magnetometer import *

from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
from Phidget22 import LogLevel


import numpy as np
import scipy.integrate as integrate
import matplotlib.pyplot as plt



#--
#-- Acceleration ISR: log acceleration changes --#
#--
def myAccelerationChangeHandler():
    print("Acceleration Change Detected.")  #BUG only enters this ISR when script is first run


#--
#-- ISR prints error code and description whenever an ERROR even occurs --#
#--
def myErrorHandler(code, description):
    print("Code: " + str(code))
    print("Description: " + description)


#--
#-- Return an initialized Accelerometer object --#
#--
def make_acceleromter_magnetometer_obj():

        
    tmp_acceleromter = Accelerometer()                      #-- Create the Phidgets.Accelerometer object --#
    tmp_magnetometer = Magnetometer()

    tmp_acceleromter.setDeviceSerialNumber(-1)              #-- Use first detected IMU serial num --#    
    tmp_magnetometer.setDeviceSerialNumber(-1)

    tmp_acceleromter.setHubPort(-1)                         #-- Use port that first detects the IMU --#
    tmp_magnetometer.setHubPort(-1) 

    tmp_acceleromter.setChannel(-1)                         #-- Use first available IMU channel --#
    tmp_magnetometer.setChannel(-1)

    tmp_acceleromter.openWaitForAttachment(500)             #-- Program will timeout if the IMU hasn't been connected after 500ms; Opens channel once connected --#  
    tmp_magnetometer.openWaitForAttachment(500)

    #tmp_acceleromter.setAccelerationChangeTrigger(0.005)      #-- setting this trigger is a rudimentary way to filter noise --#
    #tmp_magnetometer.setMagneticFieldChangeTrigger(0.005)

    # tmp_acceleromter.setOnAccelerationChangeHandler(      #-- Handler that will be called when the AccelerationChange event occurs --#
    #    myAccelerationChangeHandler())   

    # tmp_acceleromter.setDataInterval(                     #-- minimum data interval is 4ms (automatic averaging won't occur at this value). --#
    #     tmp_acceleromter.getMaxDataInterval())                 



    # NOTE: CompassCorrectionParameters are unique to each Phidget, and are affected by physical location, hard/ soft Iron offsets, and bias errors.
    #tmp_magnetometer.setCorrectionParameters(magneticField, offset0, offset1, offset2, gain0, gain1, gain2, T0, T1, T2, T3, T4, T5)

    return(tmp_acceleromter, tmp_magnetometer)


#-- 
#-- Defeat gravity! around x-axis => roll, around y-axis => pitch
#-- 

rad_to_deg = 180/np.pi

def calc_static_roll_pitch_yaw(g_x, g_y, g_z, m_x, m_y, m_z):
    #tilt = np.array([np.arcsin(g_x), np.arcsin(g_y)], np.arcsin)

    #-- ROLL -##
    roll = np.arctan2(g_y, g_z)

    #-- PITCH --#
    pitch = np.arctan( -g_x/(g_y*np.sin(roll) + g_z*np.cos(roll)) )
    
    #-- YAW --#
    numerator   = m_z * np.sin(roll) - m_y * np.cos(roll)
    denominator = m_x * np.cos(pitch) + m_y * np.sin(pitch) * np.sin(roll) + m_z * np.sin(pitch) * np.cos(roll)
    yaw = np.arctan2(numerator, denominator)
    
    # double yawAngle = Math.Atan2(magField[2] * Math.Sin(rollAngle) - magField[1] * Math.Cos(rollAngle),
	# 	magField[0] * Math.Cos(pitchAngle) + magField[1] * Math.Sin(pitchAngle) * Math.Sin(rollAngle) + magField[2] * Math.Sin(pitchAngle) * Math.Cos(rollAngle))

    tilt = np.array([roll, pitch, yaw])

    return tilt*rad_to_deg





#--
#-- Reset current position to "home" position (take sample of values, store, and subtract form)
#--




#--
#-- Print Acceleration and Magnetic field readings to terminal --#
#--
def print_data(accelerationVector, magneticFieldVector):
    print("\nAcceleration Values:\t{}\n" 
        "Magnetic Field Values:\t{}\n".format(accelerationVector, magneticFieldVector))


#--
#-- Rotate 3D vector by roll, pitch
#--
                                            # NOTE: reference, https://scipython.com/book/chapter-6-numpy/examples/creating-a-rotation-matrix-in-numpy/
def rotate_vector(input_vector, roll, pitch):

    x_in, y_in, z_in = input_vector
    input_vector = np.matrix([(x_in), (y_in), (z_in)])

    theta_x = np.radians(roll)
    theta_y = np.radians(pitch)

    c_x, s_x = np.cos(theta_x), np.sin(theta_x)
    c_y, s_y = np.cos(theta_x), np.sin(theta_y)


    R_x = np.matrix( [(1, 0, 0), (0, c_x, -s_x), (0, s_x, c_x)] )    #-- Rotation Mat about x-axis (roll) --#
    R_y = np.matrix( [(c_y, 0, s_y), (0, 1, 0), (-s_y, 0, c_y)] )    #-- Rotation Mat about y-axis (yaw) --#

    return np.dot(np.dot(R_x, R_y), input_vector.transpose())        #-- R_x . R_y . input_vector = output_vector --#    








def main():
    try:

            #-- Setup IMU accelerometer/ magnetometer objs --#
        myIMU       = make_acceleromter_magnetometer_obj()
        IMU_acc     = myIMU[0]
        IMU_mag     = myIMU[1]


        while True:

                #-- Print raw values --#
            raw_acc, raw_mag = IMU_acc.getAcceleration(), IMU_mag.getMagneticField()
            print_data(raw_acc, raw_mag)
        
                #-- Print roll, pitch, and yaw --#
            roll, pitch, yaw = calc_static_roll_pitch_yaw(IMU_acc.getAcceleration()[0], IMU_acc.getAcceleration()[1], IMU_acc.getAcceleration()[2], IMU_mag.getMagneticField()[0], IMU_mag.getMagneticField()[1], IMU_mag.getMagneticField()[2])
            print("current Roll, Pitch, and Yaw: [{} deg, {} deg, {} deg]\n".format(roll, pitch, yaw))
            

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nUser closed program.")
        IMU_acc.close()
        IMU_mag.close()
if __name__ == '__main__':
    main()