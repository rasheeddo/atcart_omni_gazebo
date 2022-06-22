#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomTF:

	def __init__(self):

		rospy.init_node("odom_tf_node", anonymous=True)

		rospy.loginfo("Publish odom frame on TF")
		self.br = tf2_ros.TransformBroadcaster()
		self.t = TransformStamped()

		rospy.Subscriber("/odom", Odometry, self.odom_callback)


		rospy.spin()

	def odom_callback(self, msg):

		self.t.header.frame_id = "odom" 
		self.t.header.stamp = msg.header.stamp #rospy.Time.now()
		self.t.child_frame_id = "base_link"	
		self.t.transform.translation.x = msg.pose.pose.position.x
		self.t.transform.translation.y = msg.pose.pose.position.y
		self.t.transform.translation.z = msg.pose.pose.position.z

		self.t.transform.rotation.x = msg.pose.pose.orientation.x
		self.t.transform.rotation.y = msg.pose.pose.orientation.y
		self.t.transform.rotation.z = msg.pose.pose.orientation.z
		self.t.transform.rotation.w = msg.pose.pose.orientation.w
		self.br.sendTransform(self.t)


if __name__ == "__main__":

	a = OdomTF()