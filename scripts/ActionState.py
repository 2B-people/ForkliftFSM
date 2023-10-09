#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import actionlib
from ForkliftFSM.msg import PurePursuitAction,PurePursuitGoal


class PurePursuitstate(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'], input_keys=['start_x', 'start_y'])
		self.client = actionlib.SimpleActionClient('pure_pursuit', PurePursuitAction)
		self.client.wait_for_server()

	def execute(self, userdata):
		"""
		Sends a goal to the PurePursuitAction server and waits for the result.

		Args:
		- userdata: user-defined data passed between states

		Returns:
		- 'succeeded' if the goal is successfully completed
		- 'preempted' if the state is preempted by another state
		- 'failed' if the goal is aborted
		"""
		goal = PurePursuitGoal()
		goal.end_xy = [userdata.start_x,userdata.start_y]
		self.client.send_goal(goal)
		rospy.loginfo("Sending goal: {goal}".format(goal=goal))
		self.client.wait_for_result()

		# 定期检查是否有抢占请求
		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				self.client.cancel_goal()
				self.service_preempt()
				return 'preempted'

			if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
				return 'succeeded'
			elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
				return 'failed'

			rospy.sleep(0.1)


