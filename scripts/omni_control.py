#!/usr/bin/env python3

import rospy
import numpy as np
from numpy import pi

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

class AtcartOmni:

	def __init__(self):
		rospy.init_node("atcart_omni_controller", anonymous=True)

		self.front_left_drive_pub = rospy.Publisher("/atcart_omni/front_left_drive_wheel_velocity_controller/command", Float64, queue_size=10)
		self.front_right_drive_pub = rospy.Publisher("/atcart_omni/front_right_drive_wheel_velocity_controller/command", Float64, queue_size=10)
		self.rear_left_drive_pub = rospy.Publisher("/atcart_omni/rear_left_drive_wheel_velocity_controller/command", Float64, queue_size=10)
		self.rear_right_drive_pub = rospy.Publisher("/atcart_omni/rear_right_drive_wheel_velocity_controller/command", Float64, queue_size=10)

		self.front_left_steer_pub = rospy.Publisher("/atcart_omni/front_left_steering_wheel_position_controller/command", Float64, queue_size=10)
		self.front_right_steer_pub = rospy.Publisher("/atcart_omni/front_right_steering_wheel_position_controller/command", Float64, queue_size=10)
		self.rear_left_steer_pub = rospy.Publisher("/atcart_omni/rear_left_steering_wheel_position_controller/command", Float64, queue_size=10)
		self.rear_right_steer_pub = rospy.Publisher("/atcart_omni/rear_right_steering_wheel_position_controller/command", Float64, queue_size=10)

		self.front_left_drive_msg = Float64()
		self.front_right_drive_msg = Float64()
		self.rear_left_drive_msg = Float64()
		self.rear_right_drive_msg = Float64()

		self.front_left_steer_msg = Float64()
		self.front_right_steer_msg = Float64()
		self.rear_left_steer_msg = Float64()
		self.rear_right_steer_msg = Float64()

		rospy.Subscriber("/joy", Joy, self.joy_callback)

		rospy.Subscriber("/odom", Odometry, self.odom_callback)

		self.left_diag_button = 0
		self.right_diag_button = 0
		self.left_90_button = 0
		self.right_90_button = 0

		self.thr = 0.0
		self.str = 0.0
		self.str_db = 0.001

		## Car paramters
		self.R_wheel = 0.15 # meter
		self.L = 0.590
		self.T = 0.530

		###########
		## TODO ###
		###########
		## Specify the max of Vx and Wz, which makes sense
		self.Vx_max = 2.0
		self.Wz_max = 3.0
		self.smallest_R = self.Vx_max/self.Wz_max
		self.biggest_R = self.Vx_max/0.2
		self.max_alp = np.arctan((self.L/2)/self.smallest_R)
		self.min_alp = np.arctan((self.L/2)/self.biggest_R)

		self.Vx = 0.0
		self.Wz = 0.0

		self.left_diag_button = 0
		self.right_diag_button = 0
		self.sideway_button = 0
		self.skid_button = 0

		self.left_diag_ang = np.arctan(self.T/self.L)
		self.right_diag_ang = -np.arctan(self.T/self.L)

		### subscribe on odom to check the feedback of linear/angular velocities of the cart
		self.vx_odom = 0.0
		self.vy_odom = 0.0
		self.v_abs_odom = 0.0
		self.wz_odom = 0.0

		self.thr = 0
		self.str = 0

		self.run()

		rospy.spin()


	def joy_callback(self, msg):

		self.left_diag_button = msg.buttons[4] #LB
		self.right_diag_button = msg.buttons[5] #RB
		self.sideway_button = msg.buttons[3] #Y
		self.skid_button = msg.buttons[1]

		self.thr = msg.axes[1]
		self.str = msg.axes[3]

	def odom_callback(self, msg):
		self.vx_odom = msg.twist.twist.linear.x
		self.vy_odom = msg.twist.twist.linear.y
		self.v_abs_odom = np.sqrt(self.vx_odom**2 + self.vy_odom**2)
		self.wz_odom = msg.twist.twist.angular.z

	def map(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is supposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter

		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		return out

		
	def run(self):

		rate = rospy.Rate(20)
		alp = 0
		front_left_drive_sign = 1.0
		front_right_drive_sign = 1.0
		rear_left_drive_sign = 1.0
		rear_right_drive_sign = 1.0
		VL = 0.0
		VR = 0.0
		Vx = 0.0
		Wz = 0.0
		R_icc = 0.0
		special_steer_flag = False

		while not rospy.is_shutdown():

			Vx = self.map(self.thr, -1.0, 1.0, -self.Vx_max, self.Vx_max)

			if self.str > 0.0:
				alp = self.map(self.str, self.str_db, 1.0, self.min_alp, self.max_alp)
				RO = (self.L/2)/np.tan(alp)
				# RO = self.map(self.str, self.str_db, 1.0, self.biggest_R, self.smallest_R)
			else:
				alp = self.map(self.str, -1.0, -self.str_db, -self.max_alp, -self.min_alp)
				# RO = self.map(self.str, -1.0, -self.str_db, -self.smallest_R, -self.biggest_R)
				RO = (self.L/2)/np.tan(alp)

			Wz = Vx/RO

			if self.left_diag_button:
				front_left_steer = self.left_diag_ang #np.radians(45)
				front_right_steer = self.left_diag_ang#np.radians(45)
				rear_left_steer = self.left_diag_ang #np.radians(45)
				rear_right_steer = self.left_diag_ang #np.radians(45)

				front_left_drive_sign = 1.0
				front_right_drive_sign = 1.0
				rear_left_drive_sign = 1.0
				rear_right_drive_sign = 1.0

				special_steer_flag = True
			elif self.right_diag_button:
				front_left_steer = self.right_diag_ang #np.radians(-45)
				front_right_steer = self.right_diag_ang #np.radians(-45)
				rear_left_steer = self.right_diag_ang #np.radians(-45)
				rear_right_steer = self.right_diag_ang #np.radians(-45)

				front_left_drive_sign = 1.0
				front_right_drive_sign = 1.0
				rear_left_drive_sign = 1.0
				rear_right_drive_sign = 1.0

				special_steer_flag = True
			elif self.sideway_button:
				front_left_steer = np.radians(90)
				front_right_steer = np.radians(90)
				rear_left_steer = np.radians(90)
				rear_right_steer = np.radians(90)

				front_left_drive_sign = 1.0
				front_right_drive_sign = 1.0
				rear_left_drive_sign = 1.0
				rear_right_drive_sign = 1.0

				special_steer_flag = True
			elif self.skid_button:
				front_left_steer = -(np.pi/2 - self.left_diag_ang)
				front_right_steer = (np.pi/2 - self.left_diag_ang)
				rear_left_steer = (np.pi/2 - self.left_diag_ang)
				rear_right_steer = -(np.pi/2 - self.left_diag_ang)

				front_left_drive_sign = 1.0
				front_right_drive_sign = -1.0
				rear_left_drive_sign = 1.0
				rear_right_drive_sign = -1.0

				special_steer_flag = True
			else:
				front_left_drive_sign = 1.0
				front_right_drive_sign = 1.0
				rear_left_drive_sign = 1.0
				rear_right_drive_sign = 1.0
				special_steer_flag = False

				## User push both throttle and steering sticks
				## The motion will be curvy
				if (abs(self.str) > self.str_db) and (abs(self.thr) > self.str_db):

					# Wz = self.map(self.str, -1.0, 1.0, -self.Wz_max, self.Wz_max)
					# RO = Vx/Wz # R_icc

					Vx_sign = abs(Vx)/Vx

					RA = np.sqrt((RO - (self.T/2.0))**2 + (self.L/2.0)**2)
					RB = np.sqrt((RO + (self.T/2.0))**2 + (self.L/2.0)**2)
					RC = RA
					RD = RB

					alp_A = np.arctan((self.L/2.0)/(RO-(self.T/2.0)))
					alp_B = np.arctan((self.L/2.0)/(RO+(self.T/2.0)))
					alp_C = -alp_A
					alp_D = -alp_B

					front_left_steer = alp_A
					front_right_steer = alp_B
					rear_left_steer = alp_C
					rear_right_steer = alp_D

					## front and rear should be same speed on the same side
					V_left = Vx_sign*abs(Wz)*RA
					V_right = Vx_sign*abs(Wz)*RB

					WL = V_left/self.R_wheel
					WR = V_right/self.R_wheel

				elif (abs(self.str) > self.str_db):
					# if self.str > 0.0:
					# 	RO = self.map(self.str, 0.1, 1.0, 10.0, self.smallest_R)
					# else:
					# 	RO = self.map(self.str, -1.0, -0.1, -self.smallest_R, -10.0)

					RA = np.sqrt((RO - (self.T/2.0))**2 + (self.L/2.0)**2)
					RB = np.sqrt((RO + (self.T/2.0))**2 + (self.L/2.0)**2)
					RC = RA
					RD = RB

					alp_A = np.arctan((self.L/2.0)/(RO-(self.T/2.0)))
					alp_B = np.arctan((self.L/2.0)/(RO+(self.T/2.0)))
					alp_C = -alp_A
					alp_D = -alp_B

					front_left_steer = alp_A
					front_right_steer = alp_B
					rear_left_steer = alp_C
					rear_right_steer = alp_D

					V_left = 0.0
					V_right = 0.0

					WL = 0.0
					WR = 0.0
 

				else:

					alp = 0.0

					front_left_steer = 0
					front_right_steer = 0
					rear_left_steer = 0
					rear_right_steer = 0

					Wz = 0.0

					RO = 0.0
					RA = 0.0
					RB = 0.0

					V_left = Vx
					V_right = Vx

					WL = V_left/self.R_wheel
					WR = V_right/self.R_wheel

			if special_steer_flag:
				front_left_drive = front_left_drive_sign*(Vx/self.R_wheel)
				front_right_drive = front_right_drive_sign*(Vx/self.R_wheel)
				rear_left_drive = rear_left_drive_sign*(Vx/self.R_wheel)
				rear_right_drive = rear_right_drive_sign*(Vx/self.R_wheel)

			else:
				front_left_drive = WL
				front_right_drive = WR
				rear_left_drive = WL
				rear_right_drive = WR

			self.front_left_drive_msg.data = front_left_drive
			self.front_right_drive_msg.data = front_right_drive
			self.rear_left_drive_msg.data = rear_left_drive
			self.rear_right_drive_msg.data = rear_right_drive

			self.front_left_steer_msg.data = front_left_steer
			self.front_right_steer_msg.data = front_right_steer
			self.rear_left_steer_msg.data = rear_left_steer
			self.rear_right_steer_msg.data = rear_right_steer


			self.front_left_drive_pub.publish(self.front_left_drive_msg)
			self.front_right_drive_pub.publish(self.front_right_drive_msg)
			self.rear_left_drive_pub.publish(self.rear_left_drive_msg)
			self.rear_right_drive_pub.publish(self.rear_right_drive_msg)

			self.front_left_steer_pub.publish(self.front_left_steer_msg)
			self.front_right_steer_pub.publish(self.front_right_steer_msg)
			self.rear_left_steer_pub.publish(self.rear_left_steer_msg)
			self.rear_right_steer_pub.publish(self.rear_right_steer_msg)

			print("thr: {:.2f} str: {:.2f} alp: {:.2f} | f_l_str: {:.2f} | f_r_str: {:.2f} | RA: {:.2f} | RB: {:.2f} | RO: {:.2f} | VL: {:.2f} | VR: {:.2f} | WL: {:.2f} | WR: {:.2f} | Vx: {:.2f} | Wz: {:.2f} | v_odom: {:.2f}| wz_odom: {:.2f}".format(\
				self.thr, self.str, np.degrees(alp), \
				np.degrees(front_left_steer),\
				np.degrees(front_right_steer),\
				# np.degrees(rear_left_steer),\
				# np.degrees(rear_right_steer),\
				RA,RB,\
				RO,\
				V_left, V_right, WL, WR, Vx, Wz, self.v_abs_odom, self.wz_odom))

			rate.sleep()



if __name__ == "__main__":

	a = AtcartOmni()



