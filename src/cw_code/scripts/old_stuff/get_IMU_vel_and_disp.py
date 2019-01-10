#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
# from dynamixel_msgs.msg import JointState
import pprint
from std_msgs.msg import Float64

import numpy as np

#Can use pointers with this...
# import ctypes 


class IMU_dead_reckoning:

    def __init__(self, imu_dataFrame_topic, t_step = 1.0):
    

    #-- Initializing node --#
        rospy.init_node('linear_velocity', anonymous=True)

    #-- Storing topic name and setting rospy rate --#
        self._imu_dataFrame_topic = imu_dataFrame_topic
        self.rate = rospy.Rate(t_step)

    #-- Attributes for integrating data --#
        self._previous_x = 0.0
        self._previous_v = 0.0
        self._t_step = t_step
        

        self._linear_x_offset = 0.0
        self._offsetNotInduced = True

    #-- Initializing data vectors --#
        self._sampled_linear_acceleration    =   Vector3(0.0,0.0,0.0)
        self._sampled_angular_velocity       =   Vector3(0.0,0.0,0.0)
        self._sampled_magnetic_magnitude     =   Vector3(0.0,0.0,0.0)       # Need magnetic-mag to properly interpret rotation about "gravitational axis"

    #-- Initializing publisher/ subscriber --#
        self.initSubscriber()
        self.initPublishers()


#-- Attribute accessor functions --#
    # def get_imu_topic(self):
    #     return self._imu_dataFrame_topic
    # def get_t_step(self):
    #     return self._t_step
    # def get_previous_x(self):
    #     return self._previous_x
    # def get_previous_v(self):
    #     return self._previous_v
    # def get_sampled_linear_acceleration(self):
    #     acc_lin_x = self._sampled_linear_acceleration.x
    #     acc_lin_y = self._sampled_linear_acceleration.y
    #     acc_lin_z = self._sampled_linear_acceleration.z
    #     return [acc_lin_x, acc_lin_y, acc_lin_z]
    # def get_sampled_angular_acceleration(self):
    #     acc_ang_x = self._sampled_angular_acceleration.x
    #     acc_ang_y = self._sampled_angular_velocity.y
    #     acc_ang_z = self._sampled_angular_velocity.z
    #     return [acc_ang_x, acc_ang_y, acc_ang_z]
    # def get_sampled_magnetic_magnitude(self):
    #     mag_x = self._sampled_magnetic_magnitude.x
    #     mag_y = self._sampled_magnetic_magnitude.y
    #     mag_z = self._sampled_magnetic_magnitude.z
    #     return [mag_x, mag_y, mag_z]



#-- Converts std_msg type Float64 to float, so measured value can be used for arithmetic --#
    def Float64_to_float(self, input):
        return float(str(input))


#-- Update running integrals --#
    # def update_running_integrals(self, acc, t_step):
    #     print(acc)
    #     new_v = self._previous_v + acc/self._t_step   # Update Running acceleration Integral.
    #     self._previous_v = new_v                # Update previous velocity.
        
    #     new_x = self._previous_x + new_v/self._t_step # Update Running velocity Integral.
    #     self._previous_x = new_x                # Update previous Displacement.


    def initSubscriber(self): 
        rospy.Subscriber(self._imu_dataFrame_topic, Imu, self.read_dataFrame)


#-- Reads contents of IMU data-packet --# 
    def read_dataFrame(self, msg):

            if abs(msg.linear_acceleration.x) <= 1.0:
                self._sampled_linear_acceleration.x = 0.0
            else:
                self._sampled_linear_acceleration.x =   msg.linear_acceleration.x
            #self._sampled_linear_acceleration.y =   msg.linear_acceleration.y
            #self._sampled_linear_acceleration.z =   msg.linear_acceleration.z

            if self._offsetNotInduced:
                self._linear_x_offset = self._sampled_linear_acceleration.x
                self._offsetNotInduced = False
            print("Linear x-offset: {}".format(self._linear_x_offset))
            print("Sampled linear x Acceleration: {}".format(self._sampled_linear_acceleration.x))



            self._previous_v += (self._sampled_linear_acceleration.x) /self._t_step   # Update Running acceleration Integral.
            print("Previos_v: {}".format(self._previous_v))

            self._previous_x += self._previous_v/self._t_step       # Update Running velocity Integral.
            print("Previous_x: {}".format(self._previous_x))

            # self._sampled_angular_velocity.x    =   msg.angular_velocity.x
            # self._sampled_angular_velocity.y    =   msg.angular_velocity.y
            # self._sampled_angular_velocity.z    =   msg.angular_velocity.z

#-- Publishing velocity and displacement data --#
    def initPublishers(self):
        #self.pub_lin_vel    =   rospy.Publisher('/imu_frame_velocity',     Float64, queue_size='100')
        self.pub_lin_disp   =   rospy.Publisher('/imu_frame_displacement', Float64, queue_size='100') 

















try:

    my_IMU = IMU_dead_reckoning("imu/data", 1.0)

    while not rospy.is_shutdown():

        my_IMU.pub_lin_disp.publish(my_IMU._previous_x)
       # print(my_IMU._previous_x)
        my_IMU.rate.sleep()

except rospy.ROSInterruptException:
    pass


