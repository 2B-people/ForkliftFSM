#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

import smach
from smach import StateMachine, Concurrence,CBState
from smach_ros import IntrospectionServer,set_preempt_handler

from ActionState import ReLocationState,PickupState
from APIService import serviceServer
from SubStateMachine import NavStateMachine,TaskStateMachine,ChargeStateMachine

class RESETState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
	def execute(self,userdata):
		rospy.loginfo('RESET fsm')
		n = 0 
		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				self.service_preempt()
				return 'preempted'
			
			# TODO重置操作
			n+=1
			rospy.sleep(0.1)	
			if n == 10:
				return 'succeeded'
		
		#TODO: 重置状态机
		return 'failed'

class IDLEState(smach.State):
	def __init__(self,srv):
		# 该状态的转移条件
		smach.State.__init__(self, outcomes=
					   ['task_call','nav_call','reloc_call','pickup_call','charge_call','preempted','failed'])
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
			if call_condition != 'IDLE':
				rospy.loginfo('call_condition is %s',call_condition)
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
		#注意，结束后需要退出 
		return 'failed'
				
class SuperVisionState(smach.State):
	def __init__(self,srv):
		smach.State.__init__(self, outcomes=['succeeded', 'repaired', 'preempted'])
		# srv调用
		self.srv = srv
		# TODO 监控状态，这里直接用socket链接
		# TODO 确定哪些情况要强行终止系统
	
	def execute(self,userdata):
		rospy.loginfo('SuperVision state is running')

		# TODO SuperVision状态，监控整个机器人的状态

		while not rospy.is_shutdown():		
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				self.service_preempt()
				return 'preempted'
			# 响应进入维修
			call_condition = self.srv.Get_Repair()
			if call_condition == 'REPAIR_IN':
				return 'repaired'
			# TODO 如果有状态不正常，也进入维修态
			is_need_repair = False
			# TODO: 机器人状态监控
			if is_need_repair:
				self.srv.Set_Repair()
				return 'repaired'

		#注意，结束后需要退出 
		return 'succeeded'

class RepairState(smach.State):
	def __init__(self,srv):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		# srv调用
		self.srv = srv
	def execute(self,userdata):
		rospy.loginfo('Repair state is running')
		# TODO 进入维修状态
		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				self.service_preempt()
				return 'preempted'
			
			# 响应出维修，到reset重启状态机
			call_condition = self.srv.Get_Repair()
			if call_condition == 'REPAIR_OUT':
				return 'succeeded'
		return 'failed'
			

def main():
	rospy.init_node('Forkift_fsm_node')
	rospy.loginfo('Forkift_fsm_node started')

	# Create a service server
	srv = serviceServer()

	# Create a SMACH state machine
	sm_root = StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
	sm_task = StateMachine(outcomes=['succeeded', 'failed', 'preempted'])


	with sm_root:
		# 添加重启态
		StateMachine.add('RESET', RESETState(), 
				   transitions={'succeeded':'RUN_SHAPES','failed':'failed'})
		# 添加并行态（包括监视态和任务子状态机）
		shape_cc = Concurrence(outcomes=['succeeded','repaired','preempted','failed'],
							   default_outcome='repaired',
							#  当条件不再成立时，要让它杀死同级，唯一需要做的就是给并发一个“子终止回调”。
							   child_termination_cb = lambda so: True,
							   outcome_map ={'repaired':{'MAIN_MONITOR':'repaired','MAIN_TASK':'preempted'},
											 'failed':{'MAIN_TASK':'failed'},
											 'succeeded':{'MAIN_MONITOR':'succeeded','MAIN_TASK':'succeeded'},
											 'preempted':{'MAIN_MONITOR':'preempted','MAIN_TASK':'preempted'}})
		with shape_cc:
			# 添加并行态，并行task和监视状态	
			Concurrence.add('MAIN_MONITOR',SuperVisionState(srv))
			Concurrence.add('MAIN_TASK', sm_task)
			with sm_task:
				StateMachine.add('IDLE', IDLEState(srv),
					transitions={'task_call':'TASK','nav_call':'NAV_SOLE',
				  				'reloc_call':'RELOC_SOLE','pickup_call':'PICK_SOLE',
								'charge_call':'CHARGE','preempted':'preempted'
								,'failed':'failed'})
				# 调用task	
				StateMachine.add('TASK', TaskStateMachine(),
				    transitions={'succeeded':'IDLE','preempted':'preempted','failed':'failed'})
				# 调用单独的动作action
				StateMachine.add('NAV_SOLE', NavStateMachine(), 
				   transitions={'succeeded':'IDLE','preempted':'preempted','failed':'failed'})
				# 重定位
				StateMachine.add('RELOC_SOLE', ReLocationState(), 
				   transitions={'succeeded':'IDLE','preempted':'preempted','failed':'failed'})
				# 取
				StateMachine.add('PICK_SOLE', PickupState(),
				   transitions={'succeeded':'IDLE','preempted':'preempted','failed':'failed'})
				#充电
				StateMachine.add('CHARGE', ChargeStateMachine(),
				   transitions={'succeeded':'IDLE','preempted':'preempted','failed':'failed'})
				
		StateMachine.add('RUN_SHAPES',shape_cc,transitions={'repaired':'REPAIR','failed':'fail2repair','preempted':'preempted'})
		# 添加一个任务调用进入failed的处理，然后这里我先直接进入REPAIR态
		def fail2repair(ud,srv):
			srv.Set_Repair()
			return 'succeeded'
		StateMachine.add('fail2repair',CBState(fail2repair,cb_kwargs={'srv':srv},outcomes={'succeeded'}),
				   transitions={'succeeded':'REPAIR'})
		# 添加维修态
		# 通过接口调用结束状态,重启后回到idle状态
		StateMachine.add('REPAIR', RepairState(srv),
				   transitions={'succeeded':'RESET','preempted':'preempted','failed':'failed'})
		


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

