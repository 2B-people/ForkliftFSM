#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

import smach
from smach import StateMachine, Concurrence
from smach_ros import IntrospectionServer,set_preempt_handler, MonitorState

from ActionState import PurePursuitState,ReLocationState,PickupState,ChargeState
from APIService import serviceServer
from SubStateMachine import NavStateMachine,TaskStateMachine,ChargeStateMachine

class RESETState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
	def execute(self,userdata):
		rospy.loginfo('reset fsm')
		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				self.service_preempt()
				return 'preempted'
			
			#TODO: 重置状态机
			return 'succeeded'

class IDLEState(smach.State):
	def __init__(self,srv):
		# 该状态的转移条件
		smach.State.__init__(self, outcomes=
					   ['task_call','nav_call','reloc_call','pickup_call','charge_call','preempted'])
		# srv调用
		self.srv = srv

	def execute(self,userdata):
		rospy.loginfo('IDLE state is running')

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
				
class SuperVisionState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["preempted"])
		# TODO 监控状态，这里直接用socket链接
	
	def execute(self,userdata):
		rospy.loginfo('SuperVision state is running')

		# TODO SuperVision状态，监控整个机器人的状态
		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				self.service_preempt()
				return 'preempted'
			
			# TODO 出现问题这里直接退出preempted

def main():
	rospy.init_node("Forkift_fsm_node")
	rospy.loginfo("Forkift_fsm_node started")

	# Create a service server
	srv = serviceServer()

	# Create a SMACH state machine
	sm_root = StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
	sm_task = StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
	
	shape_cc = Concurrence(outcomes=['succeeded','failed','preempted'],
							   default_outcome='failed',
							   outcome_map ={'preempted':{'SuperVision':'preempted','sm_task':'preempted'}})


	with sm_root:
		# 添加重启态
		StateMachine.add('RESET', RESETState(), 
				   transitions={'succeeded':'RUN_SHAPES','preempted':'preempted'})
		# 重启后开始main循环
		with shape_cc:
			# 添加并行态，并行task和监视状态	
			Concurrence.add('SuperVision',SuperVisionState())
			Concurrence.add("sm_task", sm_task)
			with sm_task:
				StateMachine.add('IDLE', IDLEState(srv),
					transitions={'task_call':'task','nav_call':'nav_cc',
				  				'reloc_call':'reloc_cc','pickup_call':'pickup_cc',
								'charge_call':'charge_cc','preempted':'preempted'})
				# 调用task	
				StateMachine.add('task', TaskStateMachine(),
				    transitions={'succeeded':'IDLE','preempted':'preempted'})

				# 调用单独的动作action
				StateMachine.add('nav_cc', PurePursuitState(), 
				   transitions={'succeeded':'IDLE','preempted':'preempted'})
				# 重定位
				StateMachine.add('reloc_cc', ReLocationState(), 
				   transitions={'succeeded':'IDLE','preempted':'preempted'})
				# 取
				StateMachine.add('pickup_cc', PickupState(),
				   transitions={'succeeded':'IDLE','preempted':'preempted'})
				#充电
				StateMachine.add('charge_cc', ChargeState(),
				   transitions={'succeeded':'IDLE','preempted':'preempted'})

		# 并行态出问题到reset中
		StateMachine.add('RUN_SHAPES',shape_cc,transitions={'preempted':'RESET'})
	
	

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

