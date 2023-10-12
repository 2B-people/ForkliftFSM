#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import SetBool, SetBoolResponse

class serviceServer():
	def __init__(self):
		"""
		Initialize the service and register the callback function.
		Also initialize the parameter server to share parameters across the state machine.
		"""
		# 测试用的空请求
		rospy.Service("fsm_node/test_empty",Empty, self.empty_service_callback)
		# 请求任务的接口，可以使得状态机转移到任务状态
		rospy.Service("fsm_node/fsm_task",SetBool, self.task_service_callback)
		rospy.Service("fsm_node/fsm_nav",SetBool, self.nav_service_callback)
		rospy.Service("fsm_node/fsm_relocation",SetBool, self.relocation_service_callback)
		rospy.Service("fsm_node/fsm_pickup",SetBool, self.pickup_service_callback)
		rospy.Service("fsm_node/fsm_charge",SetBool, self.charge_service_callback)
		# 进入维修状态的接口，可以使得状态机终止当前任务进入维修状态
		# 进入
		rospy.Service("fsm_node/fsm_repair_in",SetBool, self.repair_in_service_callback)
		# 退出
		rospy.Service("fsm_node/fsm_repair_out",SetBool, self.repair_out_service_callback)
		# 完成service的初始化
		rospy.loginfo("service server is ready.")

		# 初始调用态为IDLE
		self.call_set = "IDLE"
		self.repair_set = "REPAIR_OUT"

		rospy.set_param("test_car_state",10)

	# 读取call_set的值
	def Get_Call(self):
		return self.call_set
	
	def Get_Repair(self):
		return self.repair_set

	# 结束调用,call_set的值设置为IDLE
	def Call_Finish(self):
		self.call_set = "IDLE"
	
	# 慎用，这里只是测试，call_set的值应该由回调函数设置
	def Set_Call(self,call_set):
		rospy.logwarn("call_set is set to %s",call_set)
		self.call_set = call_set
	
	# 任务调用改变call_set的值
	def call_change_set(self,set):
		if self.call_set == "IDLE":
			self.call_set = set
			return SetBoolResponse(True,"task request received")	
		else:
			rospy.logwarn("call task request is true, but fsm is not IDLE")	
			return SetBoolResponse(False,"call task request is true")

	# 测试用接口
	def empty_service_callback(self,request):
		rospy.loginfo("Received an empty request. Performing some action.")
		return EmptyResponse()
	# 执行任务调用，包括导航，重定位，拾取，完整执行业务
	def task_service_callback(self,request):
		if request.data:
			return self.call_change_set("TASK")
		else:
			rospy.logwarn("call task request is false")
			return SetBoolResponse(False,"call task request is false")
	# 单独动作调用
	# 导航
	def nav_service_callback(self,request):
		if request.data:
			return self.call_change_set("NAV")
		else:
			rospy.logwarn("call task request is false")
			return SetBoolResponse(False,"call task request is false")
	# 重定位
	def relocation_service_callback(self,request):
		if request.data:
			return self.call_change_set("RELOC")
		else:
			rospy.logwarn("call task request is false")
			return SetBoolResponse(False,"call task request is false")
	# 拾取
	def pickup_service_callback(self,request):
		if request.data:
			return self.call_change_set("PICKUP")
		else:
			rospy.logwarn("call task request is false")
			return SetBoolResponse(False,"call task request is false")
	# 充电
	def charge_service_callback(self,request):
		if request.data:
			return self.call_change_set("CHARGE")
		else:
			rospy.logwarn("call task request is false")
			return SetBoolResponse(False,"call task request is false")
		
	# 进入维修态
	def repair_in_service_callback(self,request):
		if request.data:
			self.repair_set = "REPAIR_IN"
			return SetBoolResponse(True,"task request received")	
		else:
			rospy.logwarn("call task request is false")
			return SetBoolResponse(False,"call task request is false")

	#退出维修态
	def	 repair_out_service_callback(self,request):
		if request.data:
			self.repair_set = "REPAIR_OUT"
			return SetBoolResponse(True,"task request received")	
		else:
			rospy.logwarn("call task request is false")
			return SetBoolResponse(False,"call task request is false")
