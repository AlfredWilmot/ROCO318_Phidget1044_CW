#!/usr/bin/env python
import os, sys
import time
import traceback
import pprint
import numpy as np

#-- For parsing csv files --#
import pandas as pd


#-- Phidget API --#
from Phidget22.Devices.Accelerometer import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
# from Phidget22.Net import *

# USER MUST PASS DESIRED SAMPLING FREQUENCY, AND SAMPLING DURATION.
# A CSV FILE CONTAINING THE ACCELEROMETER DATA IS GENERATED.``





############################################
##-- Accelerometer object initialization --#
############################################
def make_acceleromter_obj(data_interval):
    tmp = Accelerometer()
    tmp.setDeviceSerialNumber(-1)    # (-1 => first relevant and present element detected will be used).
    tmp.setHubPort(-1)
    tmp.setChannel(-1) 
    tmp.openWaitForAttachment(500)          # Timeout.


    tmp.setDataInterval(data_interval)      # setting the data interval.


    return(tmp)
##########################################





##############################################################
#-- prints some basic info about the connected Phidget IMU --#
#############################################################
def print_imu_analytics(IMU_obj):

    

    min_data_interval       = IMU_obj.getMinDataInterval()
    max_data_interval       = IMU_obj.getMaxDataInterval()
    current_data_interval   = IMU_obj.getDataInterval()
    min_acceleration        = IMU_obj.getMinAcceleration()
    mac_acceleration        = IMU_obj.getMaxAcceleration()

    print("Min data Interval: {} ms".format(min_data_interval))
    print("Max data Interval: {} ms".format(max_data_interval))
    print("Current data Interval set to: {} ms".format(current_data_interval))
    print("Min acceleration (according to SDK): {} g".format(min_acceleration))
    print("Max acceleration (according to SDK): {} g".format(mac_acceleration))
#############################################################





##############################################################################
##-- Preliminary experiment code: deliniates between single-f, and f-sweep --#
##############################################################################
def prepare_test(data_interval, interval_step, max_samples, IMU_id, dir_path):

##-- Initialize time-stamp --#
    tmp_time_stamp = 0

#-- Creating a new IMU object, with a new data_interval --#
    IMU_obj = make_acceleromter_obj(data_interval)


#-- Single frequency sampling case. --#
    if interval_step == 0:

        gather_and_store_data(IMU_obj, data_interval, max_samples, IMU_id, dir_path)

#-- Frequency sweep case. --#
    else: 

    #-- Sweep through sampling frequencies, starting at lowest data interval (4ms), ending near highest (1000ms) --#
        for dt_intv in range(data_interval, 1000, interval_step):
            
            gather_and_store_data(IMU_obj, dt_intv, max_samples, IMU_id, dir_path)

######################################################################
  
    
    


#####################################################################################################
##-- Core Experiment code, stores realtime data in an array, then generates a csv from that data --##
#####################################################################################################
def gather_and_store_data(IMU_obj, dt_intv, max_samples, IMU_id, dir_path):

#-- List stores time against sampled data (reset for each round in for-loop)--#
    time_and_data = []

#-- Adjusting IMU data_interval --#
    IMU_obj.setDataInterval(dt_intv)

#-- First few values will be trimmed so dataset represents a stable signal --# 
    trim = 5

    ############################################
    while(len(time_and_data) < max_samples + trim):
    
    #-- storing data & time-stamp temporarily --#
        tmp_data        =    IMU_obj.getAcceleration()
        tmp_time_stamp  =    IMU_obj.getTimestamp()

    #-- Adding info to the list --#
        time_and_data.append([tmp_time_stamp, tmp_data])

    #-- waiting until ready to store next sample --#
        time.sleep(dt_intv*10**(-3))
    ############################################

#-- Prepare data-frame (trimming first few values)--#
    df = pd.DataFrame(time_and_data[trim:], columns=["TimeStamp_(ms)", ["acc(_x_(g)", "acc_y_(g)", "acc_z_(g)"]])

#-- Store data-frame --#
    output_csv_name = "imuID_{} samplingRate_{}ms".format(IMU_id, dt_intv)

    print("\n\n{}\n\n".format(output_csv_name))

    df.to_csv(dir_path + output_csv_name + ".csv", index=False)
#####################################################################################################





#########################################################
##-- Returns the expected duration of the experiment --##
#########################################################
def calc_experiment_duration(min_data_interval, max_data_interval, interval_step, samples_per_step):
    
    #-- Single frequency sampling case. --#
    if (interval_step == 0):

        tmp = min_data_interval * samples_per_step * (10**-3)/60

    #-- Frequency sweep case. --#
    else:
    
        tmp = np.array(range(min_data_interval, max_data_interval, interval_step))
        tmp = tmp*samples_per_step
        tmp = int(tmp.sum() * (10**-3)/60)
    
    print("The experiment will last {} minutes".format(tmp))

    query = raw_input("\n\nCONTINUE (y/n)?  ")

    if query == 'y':
        pass
    else:
        raise ValueError("\n\nUser has aborted the experiment as it will take too long.\n\n")
#########################################################





###################################################
##-- Creates missing directory if it's missing --##
###################################################
def mkdir_if_empty(dir_path):
    
    try:
    
        out_dir = ""

        dir_tree = dir_path.split('/')

        for dir in dir_tree[:-1]:
            out_dir += dir + '/'


            print("Attempting to create dir: ' {} '".format(out_dir))

            try:
                os.mkdir(out_dir)
            except OSError as err_info:
                
                print("Parent folder (' {} ') already exists, creating child if applicable.".format(dir))
            

    except OSError as err_info:
        print(err_info)
###################################################







def main():
    try:
        
        
            usr_select = int(input("Select Experiment mode:\n(1) Single Frequency\n(2) Frequency Sweep\n... "))


            if (usr_select == 1):
                data_interval = int(input("Designate single IMU data sampling rate within range [4ms, 1000ms], in steps of 4ms\n... "))
                interval_step = 0   #Used to indicate that only want to test single sampling frequency.  
            elif (usr_select == 2):
                data_interval = 4   #4ms = lowest data_interval, 1000ms = greatest data interval.
                interval_step = int( input("Input interval-step (multiple of 4)\n... "))
            else:
                print("\nInvalid option. elect either '1' or '2'.\n")
                return -1
            
            max_samples         = int( input("Input the desired number of samples for a given frequency\n... " ))
            IMU_ID              = str( input("Input the ID given to the IMU (check underside)\n... "))
            test_ID             = raw_input("Input a unique ID for this test\n... ")        


            calc_experiment_duration(data_interval ,1000, interval_step, max_samples)

            dir_path = "csv_data/IMU_ID_{}/test_{}/".format(IMU_ID, test_ID)
            
            mkdir_if_empty(dir_path)


            #-- User can repeat the last defined experiment if desired --#
            repeat = 'y'

            while (repeat == 'y'):

                prepare_test(data_interval, interval_step, max_samples, IMU_ID, dir_path)
                repeat = raw_input("Repeat the same experiment (y/n)?\n**WILL OVERWRITE PREVIOUS EXPERIMENT DATA**\n... ")

                if (repeat !='y'):
                    return 0
                else:
                    pass 
    
            return 0





#-- React to keyboard Interrupt --#
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt pressed.")

        return -1
#-- Exception if user passes frequency or data_interval value in improper format --#
    except NameError:
        print("\n INVALID INPUT \n")
        return -1

#-- Exception when user specifies sampling interval outside the permitted bandwidth (4ms, 1000ms) --#
    except PhidgetException:
        print("\nCHECK PHIDGET IS CONNECTED TO COMPUTER.\n")
        print("\nSpecified sampling interval is outside of range [4ms, 1000ms].\n")
        return -1
        #NOTE: Specified bandwidth is: 497Hz (cannot test directly!)

#-- ValueError raised, see printed message for details --#
    except ValueError as e_msg:
        print(e_msg)
        return -1

if __name__ == '__main__':
    main()

