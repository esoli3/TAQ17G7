#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

targetB1_cascade = cv2.CascadeClassifier('/home/pi1/catkin_ws/src/kinetic_sample_packages/kinetic_sample_image_sub_py/src/kinetic_sample_image_sub_py/targetB/cascade1.xml')
targetB1_cascade = cv2.CascadeClassifier('/home/pi1/catkin_ws/src/kinetic_sample_packages/kinetic_sample_image_sub_py/src/kinetic_sample_image_sub_py/targetB/cascade2.xml')

class PoseEstimator():
	def __init__(self):
		#Set up the CV Bridge
		self.bridge = CvBridge()

		self.param_topic_camera_info = rospy.get_param("~topic_camera_info", "/raspicam_node/camera_info")
		self.param_topic_image_input = rospy.get_param("~topic_image_input", "/raspicam_node/image_raw")
		self.param_topic_debug_output = rospy.get_param("~topic_image_debug", "~debug/image_raw")
		self.param_topic_overlay_output = rospy.get_param("~topic_image_overlay", "~overlay/image_raw")
		self.param_topic_pose = rospy.get_param("~topic_pose", "~circle_pose")
		self.param_circle_radius = rospy.get_param("~circle_radius", 0.1)
		self.param_circle_h_min = rospy.get_param("~circle_h_min", 170)
		self.param_circle_s_min = rospy.get_param("~circle_s_min", 50)
		self.param_circle_v_min = rospy.get_param("~circle_v_min", 50)
		self.param_circle_h_max = rospy.get_param("~circle_h_max", 10)
		self.param_circle_s_max = rospy.get_param("~circle_s_max", 255)
		self.param_circle_v_max = rospy.get_param("~circle_v_max", 255)

		self.got_camera_info = False
		self.camera_matrix = None
		self.dist_coeffs = None

		# Generate the model for the pose solver
		# For this example, draw a square around where the circle should be
		# There are 5 points, one in the center, and one in each corner
		r = self.param_circle_radius
		self.model_object = np.array([(0.0, 0.0, 0.0),
										(r, r, 0.0),
										(r, -r, 0.0),
										(-r, r, 0.0),
										(-r, -r, 0.0)])

		# Prepare the pose output message
		self.pose_out = PoseStamped()
		# With this basic method, we can't guess orientation, so pre-fill it with no rotation
		self.pose_out.pose.orientation.w = 1.0
		self.pose_out.pose.orientation.x = 0.0
		self.pose_out.pose.orientation.y = 0.0
		self.pose_out.pose.orientation.z = 0.0

		# Set up the subscribers and publishers
		self.sub_ci = rospy.Subscriber(self.param_topic_camera_info, CameraInfo, self.callback_ci)
		self.sub_img = rospy.Subscriber(self.param_topic_image_input, Image, self.callback_img)

		self.pub_pose = rospy.Publisher(self.param_topic_pose, PoseStamped, queue_size=10)
		self.pub_debug = rospy.Publisher(self.param_topic_debug_output, Image, queue_size=1)
		self.pub_overlay = rospy.Publisher(self.param_topic_overlay_output, Image, queue_size=1)

	def shutdown(self):
		# Unregister anything that needs it here
		self.sub_ci.unregister()
		self.sub_img.unregister()

	def callback_ci(self, msg_in):
		# Copy in the camera characteristics
		self.dist_coeffs = np.array([[msg_in.D[0], msg_in.D[1], msg_in.D[2], msg_in.D[3], msg_in.D[4]]], dtype="double")

		self.camera_matrix = np.array([
                 (msg_in.P[0], msg_in.P[1], msg_in.P[2]),
                 (msg_in.P[4], msg_in.P[5], msg_in.P[6]),
                 (msg_in.P[8], msg_in.P[9], msg_in.P[10])],
				 dtype="double")

		self.pose_out.header.frame_id = msg_in.header.frame_id

		self.got_camera_info = True

	def callback_img(self, msg_in):
		#Don't bother to process image if we don't have the camera calibration
		if self.got_camera_info:
			#Convert ROS image to CV image
			try:
				cv_image = self.bridge.imgmsg_to_cv2( msg_in, "bgr8" )
				cv_image = cv2.flip(cv_image,0)
			except CvBridgeError as e:
				print(e)
			
			gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
			circles = targetB1_cascade.detectMultiScale(gray, 1.1, 3)
			circles1 = targetB2_cascade.detectMultiScale(gray, 1.1, 3)
			#Convert the image to HSV and prepare the mask
			#hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
			#mask_image = None

			#thresh_lower = np.array([self.param_circle_h_min, self.param_circle_s_min, self.param_circle_v_min])
			##thresh_upper = np.array([self.param_circle_h_max, self.param_circle_s_max, self.param_circle_v_max])

			# We need to do a wrap around HSV 180 to 0 if the user wants to mask this color
			#if self.param_circle_h_min > self.param_circle_h_max:
				#thresh_lower_wrap = np.array([180, self.param_circle_s_max, self.param_circle_v_max])
				#thresh_upper_wrap = np.array([0, self.param_circle_s_min, self.param_circle_v_min])

				#mask_lower = cv2.inRange(hsv_image, thresh_lower, thresh_lower_wrap)
				#mask_upper = cv2.inRange(hsv_image, thresh_upper_wrap, thresh_upper)

				#mask_image = cv2.bitwise_or(mask_lower, mask_upper)
			#else:
				#mask_image = cv2.inRange(hsv_image, thresh_lower, thresh_upper)

			# Refine image to get better results
			#kernel = np.ones((5,5),np.uint8)
			#mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_OPEN, kernel)

			# Find circles in the masked image
			#min_dist = mask_image.shape[0]/8
			#circles = cv2.HoughCircles(mask_image, cv2.HOUGH_GRADIENT, 1, min_dist, param1=50, param2=20, minRadius=0, maxRadius=0)
      

			# cascade 1!!!!!!!

			if circles is not None:
				have_first_circle = False

				for c in circles:
					if not have_first_circle:
						have_first_circle = True
						#print(c)

						self.pose_out.header.stamp = msg_in.header.stamp
						self.pose_out.pose.position.x = (c[0] + c[2]/2)* 0.00236
						self.pose_out.pose.position.y= (c[1] + c[3]/2)* 0.00236
						self.pose_out.pose.position.z = 1.5

						self.pub_pose.publish(self.pose_out)

						#Draw the circle
						
						for (x,y,w,h) in circles:
							cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,255,0),2)



			# Cascade 2 !!!
			if circles2 is not None:
				have_first_circle = False

				for c in circles2:
					if not have_first_circle:
						have_first_circle = True
						#print(c)

						self.pose_out.header.stamp = msg_in.header.stamp
						self.pose_out.pose.position.x = (c[0] + c[2]/2)* 0.00236
						self.pose_out.pose.position.y= (c[1] + c[3]/2)* 0.00236
						self.pose_out.pose.position.z = 1.5

						self.pub_pose.publish(self.pose_out)

						#Draw the circle
						
						for (x,y,w,h) in circles2:
							cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,255,0),2)
			#Convert CV image to ROS image and publish
			try:
				#self.pub_debug.publish( self.bridge.cv2_to_imgmsg( mask_image, "mono8" ) )
				self.pub_overlay.publish( self.bridge.cv2_to_imgmsg( cv_image, "bgr8" ) )
			except CvBridgeError as e:
				print(e)














