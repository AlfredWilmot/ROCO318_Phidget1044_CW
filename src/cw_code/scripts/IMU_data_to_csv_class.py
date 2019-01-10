



class sniffIMU():
     def __init__(data_interval, IMU_id, dir_path):




    def make_accelerometer_obj(self):
        tmp = Accelerometer()
        tmp.setDeviceSerialNumber(-1)    # (-1 => first relevant and present element detected will be used).
        tmp.setHubPort(-1)
        tmp.setChannel(-1) 
        tmp.openWaitForAttachment(500)          # Timeout.


        tmp.setDataInterval(self.data_interval)      # setting the data interval.


    return(tmp)





def main():
    try:
        
        data_interval       = 4     #4ms = lowest data_interval, 1000ms = greatest data interval.
        max_samples         = int( input("Input the desired number of samples for a given frequency: " ))
        interval_step       = int( input("Input interval-step (multiple of 4): "))
        IMU_ID              = str( input("Input the ID given to the IMU (check underside): "))
        axis_of_interest    = raw_input("Input the axis under test: ")        


        calc_experiment_duration(4 ,1000, interval_step, max_samples)

        dir_path = "../csv_data/IMU_ID_{}/{}_axis/".format(IMU_ID, axis_of_interest)
        
        mkdir_if_empty(dir_path)

        gather_data(data_interval, interval_step, max_samples, axis_of_interest, IMU_ID, dir_path)

        
        
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
