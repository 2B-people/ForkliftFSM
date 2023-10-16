#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import actionlib
from ForkliftFSM.msg import PurePursuitAction,PurePursuitGoal

# DONE
class PurePursuitState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		self.client = actionlib.SimpleActionClient('pure_pursuit', PurePursuitAction)
		# 1.等待action服务器上线
		rospy.loginfo("wait for PurePursuit action server")
		self.client.wait_for_server()
		rospy.loginfo("PurePursuit action server is online")

	def execute(self, userdata):
		rospy.loginfo("pure_pursuit state is running")

		end_x = 0.0
		end_y = 0.0

		# 2.取出目标点设置,这里只设置了end_xy,起始点不知道
		if rospy.has_param("fsm_node/end_x") and rospy.has_param("fsm_node/end_y"):
			end_x = rospy.get_param("fsm_node/end_x")	
			end_y =rospy.get_param("fsm_node/end_y")
		else:
			rospy.logerr("FSM: No end point set!")
			return 'failed'
		
		# 3.发送目标点开始执行导航任务，这里用action定义的格式
		goal = PurePursuitGoal()
		goal.end_xy = [end_x,end_y]

		# 4.状态机发送目标点，注册反馈函数
		self.client.send_goal(goal,feedback_cb=self.feedback_cb)
		rospy.loginfo("Sending goal: {goal}".format(goal=goal))

		# 循环等待被抢占或者动作执行完成
		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				rospy.logwarn("FSM: PurePursuit Action is preempted!")
				# 5.中止动作
				self.client.cancel_goal()
				self.service_preempt()
				return 'preempted'
			# 6.取到达结果
			if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
				rospy.loginfo("PurePursuit Action succeeded!")
				return 'succeeded'
			elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
				rospy.logerr("FSM: PurePursuit Action aborted!")
				return 'failed'
			
			# 10hz查询结果情况
			rospy.sleep(0.1)
		# 结束后要进入一个空状态，否则会报错
		self.service_preempt()
		return 'failed'
		
	# 回调函数，打印反馈信息
	def feedback_cb(self, feedback):
		rospy.loginfo("Received feedback: {feedback}".format(feedback=feedback))
		rospy.set_param("fsm_node/car_state",feedback.status)
		rospy.set_param("fsm_node/cur_x",feedback.cur_xy[0])
		rospy.set_param("fsm_node/cur_y",feedback.cur_xy[1])
		rospy.set_param("fsm_node/cur_theta",feedback.cur_theta)
		rospy.set_param("fsm_node/car_head",feedback.car_head)


class ReLocationState(smach.State):
	# TODO: 重定位的状态机
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		# self.client = actionlib.SimpleActionClient('re_location', PurePursuitAction)
		# self.client.wait_for_server()

	def execute(self, userdata):
		# TODO 发送action

		# 循环等待被抢占或者动作执行完成
		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				# self.client.cancel_goal()
				rospy.logwarn("FSM: ReLocation Action is preempted!")
				self.service_preempt()
				return 'preempted'

			# 下面是测试代码
			rospy.sleep(1)
			return 'succeeded'

			# TODO重定位action的状态，结束确认
			# if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
			# 	return 'succeeded'
			# elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
			# 	return 'failed'
		self.service_preempt()
		return 'failed'
		

class PickupState(smach.State):
	# TODO: 拾取的状态机
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		
		# self.client = actionlib.SimpleActionClient('re_location', PurePursuitAction)
		# self.client.wait_for_server()

	def execute(self, userdata):
		rospy.loginfo("Pickup state is running")
		
		is_pickup = 0

		# 取出目标点设置
		if rospy.has_param("fsm_node/is_pickup"):
			is_pickup = rospy.get_param("fsm_node/is_pickup")	
		else:
			rospy.logerr("FSM: No pickup set!")
			return 'failed'
		
		# TODO这里需要发送取货或者卸货的动作
		if is_pickup == 1:
			# 取货
			rospy.loginfo("Pickup")
		elif is_pickup == 2:
			#卸货
			rospy.loginfo("Dropoff")
		elif is_pickup == 0:
			# 无动作
			rospy.loginfo("No action")
		else:
			rospy.logerr("FSM: is_pickup is set %d,get error", is_pickup)
			return 'failed'

		# 循环等待被抢占或者动作执行完成
		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				# TODO这里需要取消动作
				# self.client.cancel_goal()
				rospy.logwarn("FSM: pickup Action is preempted!")
				self.service_preempt()
				return 'preempted'
			
			# 下面是测试代码
			rospy.sleep(0.1)
			return 'succeeded'

			# TODO拾取action的状态，结束确认
			# if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
			# 	return 'succeeded'
			# elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
			# 	return 'failed'
		self.service_preempt()
		return 'failed'


class ChargeState(smach.State):
	# TODO: 充电的action状态机
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'])
		# self.client = actionlib.SimpleActionClient('re_location', PurePursuitAction)
		# self.client.wait_for_server()

	def execute(self, userdata):
		# send goal	

		# 循环等待被抢占或者动作执行完成
		while not rospy.is_shutdown():
			if self.preempt_requested():
				# 如果有抢占请求，则停止当前的动作执行，并返回preempted状态
				# self.client.cancel_goal()
				rospy.logwarn("FSM: Charge Action is preempted!")
				self.service_preempt()
				return 'preempted'

			# if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
			# 	return 'succeeded'
			# elif self.client.get_state() == actionlib.GoalStatus.ABORTED:
			# 	return 'failed'

			rospy.sleep(0.1)
			return 'succeeded'
		self.service_preempt()
		return 'failed'
		