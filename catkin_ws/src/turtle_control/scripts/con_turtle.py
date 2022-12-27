#!/usr/bin/env python3
import rospy
# import gazebo_callback as gm
# from gazebo_callback import *
from std_msgs.msg import Int64
#from sensor_msgs.msg import Imu
from std_msgs.msg import Float64 as joint_command
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import NavSatFix
from udp_base.msg import wplist
# from udp_uv.msg import device
# from sensor_msgs.msg import Imu
#from imu_3dm_gx4.msg import FilterOutput
from tf.transformations	import euler_from_quaternion
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
# import Variables as vb
import pdb
import math

class Heron_Con(object):

	######################################################################################
	##                   Initialization
	######################################################################################
	def __init__(self):
		self.wp_flag = 0
		self.fin_flag = 0
		self.wp_id = -1
		self.first_iter_flag = 1
		self.prv_x = 0
		self.prv_y = 0
		self.prv_psi = 0

		self.wp_num = 0

		self.acc = 0.2

		self.output_max = 20

		self.vel_msg_left = joint_command()
		self.vel_msg_right = joint_command()
		# self.dev_info = device()
		self.m_dX = 0
		self.m_dY = 0
		self.m_dYaw = 0 #heading

		self.wp_flag = 0

		self.mode = 0
		rospy.Subscriber('/wplist', wplist, self._rcv_wp_heron)
		rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_heading)
		rospy.Subscriber('/cmd_base', Int64, self.callback_cmd)

		self.pub_left_wheel = rospy.Publisher('turtlebot3_waffle_sim/left_wheel_velocity_controller/command', joint_command, queue_size=10)
		self.pub_right_wheel = rospy.Publisher('turtlebot3_waffle_sim/right_wheel_velocity_controller/command', joint_command, queue_size=10)



	def callback_heading(self,msg):
		orientation_list = [ msg.pose[1].orientation.x, msg.pose[1].orientation.y,\
									msg.pose[1].orientation.z, msg.pose[1].orientation.w ]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
		self.m_dYaw = yaw
		self.m_dX = msg.pose[1].position.x
		self.m_dY = msg.pose[1].position.y

	def callback_cmd(self,msg):
		if self.mode is not msg.data:
			self.mode = msg.data

# -------------------- cal_con2 -------------------------------------------------------		
	def _rcv_wp_heron(self,msg):
		if self.wp_id is not msg.id:
			self.wp_cnt = 0
			self.wp_list_x = msg.x
			self.wp_list_y = msg.y
			self.wp_num = msg.size
			self.wp_id = msg.id
			self.wp_flag = 1;

	def cal_con2(self):

		gain_rho_p = 4
		gain_psi_p_little_err = 10
		gain_psi_p = gain_psi_p_little_err
		gain_psi_p_big_err = 100
		# pdb.set_trace()
		if (self.wp_cnt < self.wp_num):
			cur_x = self.m_dX  
			cur_y = self.m_dY
			cur_psi = self.m_dYaw

			if (self.first_iter_flag == 1):
				self.prv_x = cur_x
				self.prv_y = cur_y
				self.prv_psi = cur_psi
				self.first_iter_flag = 0
			
			dsrd_x = self.wp_list_x[self.wp_cnt]
			dsrd_y = self.wp_list_y[self.wp_cnt]
		
			x_err = dsrd_x - cur_x
			y_err = dsrd_y - cur_y

			rho_err = np.sqrt( np.square(x_err) + np.square(y_err) )
			dsrd_psi = np.arctan2( y_err, x_err )

			psi_err = dsrd_psi - cur_psi

			if psi_err >= np.pi:
				while psi_err >= np.pi:
					psi_err = psi_err - 2*np.pi
			elif psi_err < -np.pi:
				while psi_err < -np.pi:
					psi_err = psi_err + 2*np.pi


			if (rho_err <= self.acc):
				if (self.wp_cnt == self.wp_num - 1):
					self.fin_flag = 1
					self.wp_cnt = 0
					return

				if (self.fin_flag == 0):
					self.wp_cnt = self.wp_cnt + 1

			self.v = gain_rho_p *rho_err
			if (self.v >20):
				self.v = 20
			elif (self.v <-20):
				self.v = -20

			if (self.v <6 and self.v >= 0):
				self.v = 6
			elif (self.v >-6 and self.v < 0):
				self.v = -6




			if (np.abs(psi_err) <= np.pi/5.0):	
				gain_psi_p = gain_psi_p_little_err
			elif (np.abs(psi_err) > np.pi/5.0):
				print('--> Large Heding Error <--')
				gain_psi_p = gain_psi_p_big_err
			self.w = -gain_psi_p*psi_err

			self.vel_msg_left.data = self.v + 0.5*self.w
			self.vel_msg_right.data = self.v - 0.5*self.w

			ratio = self.output_max / np.maximum(np.absolute(self.vel_msg_left.data), np.absolute(self.vel_msg_right.data))
			
			if ((np.absolute(self.vel_msg_left.data) > self.output_max) or (np.absolute(self.vel_msg_right.data) > self.output_max)):	
				self.vel_msg_left.data = self.vel_msg_left.data*ratio
				self.vel_msg_right.data = self.vel_msg_right.data*ratio

			self.next_task = self.wp_cnt    # Modifying at 181112 
			print('=================================================')
			print('wp_cnt          : %d ' % (self.wp_cnt))
			print('wp_coordinate          : %.2f, %.2f ' % (dsrd_x, dsrd_y))
			print('distance err    : %.2f   [m]' % (rho_err))
			print('heading err 	   : %.2f   [deg] ' % (psi_err*(180.0/np.pi)))
			print('heading         : %.2f   [deg]' % (self.m_dYaw*(180.0/np.pi)))
			print('desired heading : %.2f   [deg] ' % (dsrd_psi*(180.0/np.pi)))
			print('left thrst      : %.2f ' % (self.vel_msg_left.data))
			print('right thrst     : %.2f ' % (self.vel_msg_right.data))
			self.pub_left_wheel.publish(self.vel_msg_left)
			self.pub_right_wheel.publish(self.vel_msg_right)
			self.prv_x = cur_x
			self.prv_y = cur_y
			self.prv_psi = cur_psi
	

	def main(self):
		local_rate = 5
		rate = rospy.Rate(local_rate)
		self.dt = 1/local_rate	

		while not rospy.is_shutdown():
			# self.update_dev_msg();
			# self.pub_dev_info.publish(self.dev_info)
			if (self.mode == 1 or (self.fin_flag == 1 or not self.wp_flag)):
				self.vel_msg_left.data = 0
				self.vel_msg_right.data = 0
				self.pub_left_wheel.publish(self.vel_msg_left)
				self.pub_right_wheel.publish(self.vel_msg_right)
			else :
				self.cal_con2()
			rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('con_heron_node', anonymous=True)
		my_node = Heron_Con()
		my_node.main()
		# main()
	except rospy.ROSInterruptException:
		pass
