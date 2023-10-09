#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach

from ActionState import PurePursuitstate


class NavStateMachine(smach.StateMachine):
	def __init__(self):
		# TODO: 初始化导航任务子状态机
		smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		with self:
			smach.StateMachine.add('NAV', PurePursuitstate(),
								   transitions={'succeeded': 'succeeded', 'failed': 'failed', 'preempted': 'preempted'})


class TaskStateMachine(smach.StateMachine):
	def __init__(self):
		# TODO: 初始化任务的子状态机
		smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])


class ChargeStateMachine(smach.StateMachine):
	def __init__(self):
		# TODO: 初始化充电的子状态机
		smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])