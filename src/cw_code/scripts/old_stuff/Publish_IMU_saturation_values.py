#!/usr/bin/env python

# The line above is found in all Python ROS nodes:
# ensures that the script is executed as a python script.

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
# from dynamixel_msgs.msg import JointState
import pprint

# Subscribe to imu/data_raw, and then  


class detect_limit:

    def __init__(self, imu_dataFrame_topic):

        self.imu_dataFrame_topic = imu_dataFrame_topic

        rospy.init_node('threshold_finder', anonymous=True)
        
        self.sampled_linear_acceleration = Vector3(0.0,0.0,0.0)
        self.sampled_angular_velocity = Vector3(0.0,0.0,0.0)

        self.previousMax_linear_acceleration = Vector3(0.0,0.0,0.0)
        self.previousMax_angular_velocity = Vector3(0.0,0.0,0.0)



        self.initSubscriber()
        self.initPublishers()

    def initSubscriber(self): 
        rospy.Subscriber(self.imu_dataFrame_topic, Imu ,self.parse_dataFrame)

    def parse_dataFrame(self, msg):
 
            self.sampled_linear_acceleration.x = msg.linear_acceleration.x
            self.sampled_linear_acceleration.y = msg.linear_acceleration.y
            self.sampled_linear_acceleration.z = msg.linear_acceleration.z

            self.sampled_angular_velocity.x = msg.angular_velocity.x
            self.sampled_angular_velocity.y = msg.angular_velocity.y
            self.sampled_angular_velocity.z = msg.angular_velocity.z

    def initPublishers(self):
        self.pub_1 = rospy.Publisher(self.imu_dataFrame_topic + '/max_linear_accelerations', Vector3, queue_size=5)
        self.pub_2 = rospy.Publisher(self.imu_dataFrame_topic + '/max_angular_velocitys', Vector3 , queue_size=5)


    def find_max(self):

        found_new_linear_acc_max = False
        found_new_angular_vel_max = False

        if abs(self.sampled_angular_velocity.x) > abs(self.previousMax_angular_velocity.x):
            self.previousMax_angular_velocity.x = self.sampled_angular_velocity.x
            found_new_angular_vel_max = True
            rospy.loginfo(rospy.get_caller_id() + "New max angular_velocity_x val: %s", self.previousMax_angular_velocity.x)

        if abs(self.sampled_angular_velocity.y) > abs(self.previousMax_angular_velocity.y):
            self.previousMax_angular_velocity.y = self.sampled_angular_velocity.y
            found_new_angular_vel_max = True
            rospy.loginfo(rospy.get_caller_id() + "New max angular_velocity_y val: %s", self.previousMax_angular_velocity.y)

        if abs(self.sampled_angular_velocity.z) > abs(self.previousMax_angular_velocity.z):
            self.previousMax_angular_velocity.z = self.sampled_angular_velocity.z
            found_new_angular_vel_max = True
            rospy.loginfo(rospy.get_caller_id() + "New max angular_velocity_z val: %s", self.previousMax_angular_velocity.z)


        if abs(self.sampled_linear_acceleration.x) > abs(self.previousMax_linear_acceleration.x):
            self.previousMax_linear_acceleration.x = self.sampled_linear_acceleration.x
            found_new_linear_acc_max = True
            rospy.loginfo(rospy.get_caller_id() + "New max linear_acceleration_x val: %s", self.previousMax_linear_acceleration.x)

        if abs(self.sampled_linear_acceleration.y) > abs(self.previousMax_linear_acceleration.y):
            self.previousMax_linear_acceleration.y = self.sampled_linear_acceleration.y
            found_new_linear_acc_max = True
            rospy.loginfo(rospy.get_caller_id() + "New max linear_acceleration_y val: %s", self.previousMax_linear_acceleration.y)

        if abs(self.sampled_linear_acceleration.z) > abs(self.previousMax_linear_acceleration.z):
            self.previousMax_linear_acceleration.z = self.sampled_linear_acceleration.z
            found_new_linear_acc_max = True
            rospy.loginfo(rospy.get_caller_id() + "New max linear_acceleration_z val: %s", self.previousMax_linear_acceleration.z)


        self.pub_1.publish(self.previousMax_linear_acceleration)
            
        self.pub_2.publish(self.previousMax_angular_velocity)




        

parsed_imu_data = detect_limit('imu/data_raw')

#rate = rospy.Rate(500)

while not rospy.is_shutdown():
    

    parsed_imu_data.find_max()

    #rate.sleep()