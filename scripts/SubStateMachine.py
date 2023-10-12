#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from smach import StateMachine,Concurrence

from ActionState import PurePursuitState,ReLocationState,PickupState,ChargeState


class NavStateMachine(StateMachine):
	def __init__(self):
		# TODO: 初始化导航任务子状态机
		StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		with self:
			StateMachine.add('NAV', PurePursuitState(),
								   transitions={'succeeded': 'succeeded', 'failed': 'failed', 'preempted': 'preempted'})


class TaskStateMachine(StateMachine):
	def __init__(self):
		# TODO: 初始化任务的子状态机
		StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		with self:
			StateMachine.add('nav2pickup', PurePursuitState(), 
					transitions={'succeeded':'second_loclization','preempted':'preempted'})
			StateMachine.add('second_loclization', ReLocationState(), 
					transitions={'succeeded':'pick_up','preempted':'preempted'})
			StateMachine.add('pick_up', PickupState(),
					transitions={'succeeded':'nav2dropoff','preempted':'preempted'})
			StateMachine.add('nav2dropoff', PurePursuitState(), 
					transitions={'succeeded':'drop_off','preempted':'preempted'})
			StateMachine.add('drop_off', PickupState(),
					transitions={'succeeded':'succeeded','preempted':'preempted'})
		


class ChargeStateMachine(StateMachine):
	def __init__(self):
		# TODO: 初始化充电的子状态机
		StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])