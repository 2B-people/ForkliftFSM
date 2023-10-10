#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

import smach
from smach import StateMachine, Concurrence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer,set_preempt_handler, MonitorState

from ActionState import PurePursuitState
from APIService import serviceServer


class IDLEState(smach.State,):
	def __init__(self,srv):
		# 该状态的转移条件
		smach.State.__init__(self, outcomes=
					   ['task_call','nav_call','reloc_call','pickup_call','charge_call','preempted'],
						output_keys=['start_x', 'start_y'])
		# srv调用
		self.srv = srv

	def execute(self,userdata):
		rospy.loginfo('Executing state 1')
		rospy.set_param('start_x',1)
		rospy.set_param('start_y',1)
		userdata.start_x = 1
		userdata.start_y = 1

		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				self.service_preempt()
				return 'preempted'

			call_condition= self.srv.Get_Call()
			if call_condition != "IDLE":
				rospy.loginfo("call_condition is %s",call_condition)
				if call_condition == 'TASK':
					self.srv.Call_Finish()
					return 'task_call'
				if call_condition == 'NAV':
					self.srv.Call_Finish()
					return 'nav_call'
				if call_condition == 'RELOC':
					self.srv.Call_Finish()
					return 'reloc_call'
				if call_condition == 'PICKUP':
					self.srv.Call_Finish()
					return 'pickup_call'
				if call_condition == 'CHARGE':
					self.srv.Call_Finish()
					return 'charge_call'
	
class ReSetState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
	def execute(self,userdata):
		rospy.loginfo('Executing state1')
		rospy.sleep(1)
		return 'succeeded'

def main():
	rospy.init_node("Forkift_fsm_node")
	rospy.loginfo("Forkift_fsm_node started")

	# Create a service server
	srv = serviceServer()

	# Create a SMACH state machine
	sm_root = StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
	 
	with sm_root:
		StateMachine.add('IDLE', IDLEState(srv),
					transitions={'task_call':'state1','nav_call':'nav_cc',
				  				'reloc_call':'failed','pickup_call':'failed',
								'charge_call':'failed','preempted':'preempted'})
		StateMachine.add('state1', state1(), 
				   transitions={'succeeded':'IDLE','preempted':'preempted'})
		StateMachine.add('nav_cc', PurePursuitState(), 
				   transitions={'succeeded':'IDLE','preempted':'preempted'})
	

	# Attach a SMACH introspection server
	sis = IntrospectionServer('fsm_node', sm_root, '/SM_ROOT')
	sis.start()
	
	# Set preempt handler
	set_preempt_handler(sm_root)

	# Execute SMACH tree in a separate thread so that we can ctrl-c the script
	smach_thread = threading.Thread(target = sm_root.execute)
	smach_thread.start()

	rate = rospy.Rate(10) # 10hz
	# Signal ROS shutdown (kill threads in background)
	while not rospy.is_shutdown():
		active_states = sm_root.get_active_states()
		rospy.set_param('fsm_node/active_states',active_states)
		rate.sleep()

	sis.stop()
	
	
if __name__ == '__main__':
	main()

