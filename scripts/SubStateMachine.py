#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from smach import StateMachine,Concurrence,CBState
from smach_ros import MonitorState

from ActionState import PurePursuitState,ReLocationState,PickupState,ChargeState

from std_msgs.msg import Bool

# 导航任务子状态机
class NavStateMachine(StateMachine):
	def __init__(self):
		StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		# 如果有障碍物，中断导航任务，等待障碍物消失（NAV_MONITOR_CC-WAIT_FOR_CLEAR>）
		with self:
			# 设置并发状态机，监控是否有障碍物，中断导航任务
			self.nav_monitor_cc = Concurrence(
				outcomes=['succeeded', 'interrupted', 'failed', 'preempted'],
				default_outcome = 'succeeded',
				child_termination_cb=lambda so: True,
				outcome_map={
					'succeeded': {'NAV': 'succeeded'},
					'failed':{'NAV':'failed'},
					'preempted': {'NAV': 'preempted'},
					'interrupted': {'MONITOR': 'invalid'}})
			with self.nav_monitor_cc:
				# 并发状态机的两个子状态，一个调用导航，一个监控：/is_stop_obstacle话题，看是否有障碍物
				Concurrence.add('NAV', PurePursuitState())
				# true = 有障碍物，false = 无障碍物
				# 定义监控函数，如果有障碍物，中断导航任务
				def is_stop_cb(ud,msg):
					# true == 有障碍物 == invalid，中断导航
					# false = 无障碍物 == valid，继续导航
					rospy.logwarn("FSM: car is stop,trans in WAIT_FOR_CLEAR")
					return not msg.data
				Concurrence.add('MONITOR',
								MonitorState('/is_stop_obstacle',Bool,is_stop_cb))
			# 添加并发状态机到主状态机
			StateMachine.add('NAV_MONITOR_CC', self.nav_monitor_cc,
					{'interrupted':'WAIT_FOR_CLEAR',
	  				'succeeded':'succeeded','failed':'failed','preempted':'preempted'})
			# 定义等待障碍物消失的状态
			def stop_clear_cb(ud,msg):
				if msg.data:
					rospy.set_param("fsm_node/is_stop",msg.data)
					# True == valid，跳转到自身
					return True
				# 现场清楚了障碍物，可以继续导航
				else:	
					rospy.logwarn("FSM: obstacle is clear,nav continue")
					rospy.set_param("fsm_node/is_stop",msg.data)
					return False
					# False == invalid，跳转到NAV_MONITOR_CC
			# 等待障碍物消失WAIT_FOR_CLEAR->NAV_MONITOR_CC
			StateMachine.add('WAIT_FOR_CLEAR', MonitorState('/is_stop_obstacle',Bool,
					    	stop_clear_cb),{'valid':'WAIT_FOR_CLEAR','invalid':'NAV_MONITOR_CC'})
			
# 初始化任务的子状态机
class TaskStateMachine(StateMachine):
	def __init__(self):
		StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		with self:
			# 设置取货点的参数
			StateMachine.add('set_pickup',CBState(self.set_pickup_param,outcomes=['succeeded','failed']),
					transitions={'succeeded':'nav2pickup','failed':'failed'})
			# 调用导航
			StateMachine.add('nav2pickup', NavStateMachine(), 
					transitions={'succeeded':'second_loclization','preempted':'preempted','failed':'failed'})
			# 二次定位
			StateMachine.add('second_loclization', ReLocationState(), 
					transitions={'succeeded':'pick_up','preempted':'preempted','failed':'failed'})
			# 取货
			StateMachine.add('pick_up', PickupState(),
					transitions={'succeeded':'set_dropoff','preempted':'preempted','failed':'failed'})
			# 设置卸货点的参数
			StateMachine.add('set_dropoff',CBState(self.set_dropoff_param,outcomes=['succeeded','failed']),
					transitions={'succeeded':'nav2dropoff','failed':'failed'})
			# 导航
			StateMachine.add('nav2dropoff', NavStateMachine(), 
					transitions={'succeeded':'drop_off','preempted':'preempted','failed':'failed'})
			# 卸货
			StateMachine.add('drop_off', PickupState(),
					transitions={'succeeded':'succeeded','preempted':'preempted','failed':'failed'})
			# 结束，完成子状态机调用，返回succeeded
			# 跳转到IDLE

	# CallBack函数，设置参数,由CBSate调用
	# 设置取货点的参数
	def set_pickup_param(self, userdata):
		if rospy.has_param("fsm_node/pickup_x") and rospy.has_param("fsm_node/pickup_y"):
			end_x = rospy.get_param("fsm_node/pickup_x")
			end_y = rospy.get_param("fsm_node/pickup_y")
			# 取货
			is_pickup = 1
			rospy.set_param("fsm_node/end_x",end_x)
			rospy.set_param("fsm_node/end_y",end_y)
			rospy.set_param("fsm_node/is_pickup",is_pickup)
			return 'succeeded'
		else:
			rospy.logerr("FSM: set_pickup_param failed")
			return 'failed'

	# 设置卸货点的参数
	def set_dropoff_param(self, userdata):
		if rospy.has_param("fsm_node/dropoff_x") and rospy.has_param("fsm_node/dropoff_y"):
			end_x = rospy.get_param("fsm_node/dropoff_x")
			end_y = rospy.get_param("fsm_node/dropoff_y")
			# 卸货
			is_pickup = 2
			rospy.set_param("fsm_node/end_x",end_x)
			rospy.set_param("fsm_node/end_y",end_y)
			rospy.set_param("fsm_node/is_pickup",is_pickup)
			return 'succeeded'
		else:
			rospy.logerr("FSM: set_dropoff_param failed")
			return 'failed'

# 充电的子状态机
class ChargeStateMachine(StateMachine):
	def __init__(self):
		StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		with self:
			# 设置charge的参数
			StateMachine.add('set_charge',CBState(self.set_charge_param,outcomes=['succeeded','failed']),
					transitions={'succeeded':'nav2charge','failed':'failed'})
			# 调用导航
			StateMachine.add('nav2charge', NavStateMachine(), 
					transitions={'succeeded':'charge','preempted':'preempted'})
			# TODO，充电需不需要二次定位
			# TODO，这里需不需要在监控充电的情况
			# 调用充电
			StateMachine.add('charge', ChargeState(), 
					transitions={'succeeded':'succeeded','preempted':'preempted'})

	#设置charge的参数 
	def set_charge_param(self, userdata):
		if rospy.has_param("fsm_node/charge_x") and rospy.has_param("fsm_node/charge_y"):
			end_x = rospy.get_param("fsm_node/charge_x")
			end_y = rospy.get_param("fsm_node/charge_y")
			rospy.set_param("fsm_node/end_x",end_x)
			rospy.set_param("fsm_node/end_y",end_y)
			return 'succeeded'
		else:
			rospy.logerr("FSM: set_charge_param failed")
			return 'failed'