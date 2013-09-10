#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
PKG = 'mission_controller'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf

# ROS messages.
#from nav_msgs.msg import Odometry
#from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from mission_controller.msg import posex

#import pdb
#pdb.set_trace()

class MissionController():
    def __init__(self):
        self.got_new_msg = False
        self.missioncontroller_msg = posex()

        # Create subscribers and publishers.
	sub_robot_pose_ekf = rospy.Subscriber ( "robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.robot_pose_ekf_callback)
        #sub_imu   = rospy.Subscriber("imu", Imu, self.imu_callback)
        #sub_odom  = rospy.Subscriber("odom", Odometry, self.odom_callback)
        # pub_pose = rospy.Publisher( "pose", PoseWithCovarianceStamped)
	pub_posex = rospy.Publisher( "x", posex)


	# Robot_pose_ekf callback function -- ???????? pose.position?
	def robot_pose_ekf_callback(self, msg):
		x = msg.pose.pose.position.x
		#y = msg.pose.pose.position.y
		#z = msg.pose.pose.position.z
		self.missioncontroller_msg.x = x
		self.got_new_msg = True


	 # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_posex.publish(self.missioncontroller.msg)
                self.got_new_msg = False



# Odometry callback function.
    #def odom_callback(self, msg):.

    # IMU callback function.
    # def imu_callback(self, msg):


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('mission_controller')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        r = MissionController()
    except rospy.ROSInterruptException: pass
