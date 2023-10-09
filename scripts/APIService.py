#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
from std_srvs.srv import Empty, EmptyResponse

class serviceServer():
	def __init__(self):
		"""
		Initialize the service and register the callback function.
		Also initialize the parameter server to share parameters across the state machine.
		"""
		rospy.Service("fsm_node/test_empty",Empty, self.empty_service_callback)
		rospy.loginfo("Empty service server is ready.")
		rospy.set_param("test_car_state",10)

	
	def empty_service_callback(request):
		"""
		Callback function for the empty service.

		This function is called when an empty service request is received. It logs a message indicating that an empty request
		was received, performs some action, and returns an empty response.

		Args:
			request: An empty request object.

		Returns:
			An empty response object.
		"""
		rospy.loginfo("Received an empty request. Performing some action.")
		return EmptyResponse()
	



	

