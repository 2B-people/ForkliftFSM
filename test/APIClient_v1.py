#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty,SetBool


class CallFSM():
    def __init__(self):
        # service的初始化
        # 测试用的空请求
        self.call_empty = rospy.ServiceProxy('fsm_node/test_empty', Empty)
        self.call_empty.wait_for_service()
        # 维修处理的接口
        # 进入维修状态
        self.call_repair_in = rospy.ServiceProxy('fsm_node/fsm_repair_in', SetBool)
        self.call_repair_in.wait_for_service()
        # 退出维修状态
        self.call_repair_out = rospy.ServiceProxy('fsm_node/fsm_repair_out', SetBool)
        self.call_repair_out.wait_for_service()
        # 请求任务的接口
        # idle->task
        self.call_task = rospy.ServiceProxy('fsm_node/fsm_task', SetBool)
        self.call_task.wait_for_service()
        #独立调用action的接口
        # idle->nav
        self.call_nav = rospy.ServiceProxy('fsm_node/fsm_nav', SetBool)
        self.call_nav.wait_for_service()
        # idle->relocation
        self.call_relocation = rospy.ServiceProxy('fsm_node/fsm_relocation', SetBool)
        self.call_relocation.wait_for_service()
        # idle->pickup
        self.call_pickup = rospy.ServiceProxy('fsm_node/fsm_pickup', SetBool)
        self.call_pickup.wait_for_service()
        # idle->charge
        self.call_charge = rospy.ServiceProxy('fsm_node/fsm_charge', SetBool)
        self.call_charge.wait_for_service()

    # 使用ros的service接口调用状态机的测试接口
    # 这里用了一个不带参数的空请求进行测试
    def CallTestEmpty(self):
        """
        调用测试接口
        """
        try:
            # 测试 service通信是否可行，这里只是发送一个空请求
            # 在状态机端会打印一条日志
            # rospy.loginfo("Received an empty request. Performing some action.")
            response = self.call_empty()
            rospy.loginfo("send req")
            return response
        except rospy.ServiceException as e:
            rospy.logerr(" TestEmpty Service call failed:")
            return None
    # 维修进入接口
    def CallRepairIn(self):
        try:
            response = self.call_repair_in(True)
            return response
        except rospy.ServiceException as e:
            rospy.logerr(" Task Service call failed:")
            return None
    # 维修出接口
    def CallRepairOut(self):
        try:
            response = self.call_repair_out(True)
            return response
        except rospy.ServiceException as e:
            rospy.logerr(" Task Service call failed:")
            return None
    
    #调用任务接口
    def CallTask(self,pickup_xy,dropoff_xy):
        """
        Calls the task interface with the given pickup and dropoff coordinates.
        Args:
            pickup_xy (tuple): A tuple containing the x and y coordinates of the pickup point.
            dropoff_xy (tuple): A tuple containing the x and y coordinates of the dropoff point.
        Returns:
            The response from the task service call, or None if the call failed.
        """
        try:
            # 设置目标，这里只用设置取货点和卸货点，货物卸载后结束该状态，返回IDLE
            rospy.set_param("fsm_node/pickup_x",pickup_xy[0])
            rospy.set_param("fsm_node/pickup_y",pickup_xy[1])
            rospy.set_param("fsm_node/dropoff_x",dropoff_xy[0])
            rospy.set_param("fsm_node/dropoff_y",dropoff_xy[1])
            # call 调用
            response = self.call_task(True)
            return response
        except rospy.ServiceException as e:
            rospy.logerr(" Task Service call failed:")
            return None
    # 调用导航动作接口
    def CallNav(self,end_xy):
        """
        Calls the navigation interface with the given end coordinates.
        Args:
            end_xy (tuple): A tuple containing the x and y coordinates of the end point.
        Returns:
            The response from the navigation service call, or None if the call failed.
        """
        try:
            # 设置导航目标
            rospy.set_param("fsm_node/end_x",end_xy[0])
            rospy.set_param("fsm_node/end_y",end_xy[1])
            # call 调用
            response = self.call_nav(True)
            return response                     
        except rospy.ServiceException as e:
            rospy.logerr(" Nav Service call failed:")
            return None
    # 调用二次定位接口
    def CallRelocation(self):
        """
        Calls the relocation interface.
        Returns:
            The response from the relocation service call, or None if the call failed.
        """
        try:
            response = self.call_relocation(True)
            return response
        except rospy.ServiceException as e:
            rospy.logerr(" Relocation Service call failed:")
            return None
    # 调用取货接口
    def CallPickup(self,is_pickup): 
        """
        Calls the pickup interface.
        Args:
            is_pickup (uint8): 
            1 if the robot should pick up the object, 1取货
            2 if it should drop it off.2卸货
        Returns:
            The response from the pickup service call, or None if the call failed.
        """
        try:
            # 设置参数,1取货，2卸货
            rospy.set_param("fsm_node/is_pickup",is_pickup)
            # call 调用
            response = self.call_pickup(True)
            return response
        except rospy.ServiceException as e:
            rospy.logerr(" Pickup Service call failed:")
            return None
    # 调用充电接口 
    def CallCharge(self):
        """
        Calls the charge interface.
        Returns:
            The response from the charge service call, or None if the call failed.
        """
        try:
            if rospy.has_param("fsm_node/charge_x") and rospy.has_param("fsm_node/charge_y"):
                response = self.call_charge(True)
                return response
            else:
                rospy.logerr(" Charge point not set")
        except rospy.ServiceException as e:
            rospy.logerr(" Charge Service call failed:")
            return None
    # 设置充电点
    def Set_Charge_point(self,charge_xy): 
        # 充电口所在位置
        rospy.loginfo("charge point is set to (%f,%f)",charge_xy[0],charge_xy[1])
        # 写到参数服务器中
        rospy.set_param("fsm_node/charge_x",charge_xy[0])
        rospy.set_param("fsm_node/charge_y",charge_xy[1])
        return True

    # 使用ros的参数服务器状态机提供接口读取叉车的状态
    # 注意，这里禁止设置参数，只能读取参数，需要修改，需要调用状态机的srv接口
    # 参数由驱动层和FSM提供，驱动层数据
    # fsm的监测状态通过TCP接口读取，由ros的参数服务器转发
    # fsm本身的数据直接写到参数服务器中
    # 上层应用可以用参数服务器得到
    def get_param(self, name, default):
        """
        获取参数，如果参数不存在则返回默认值
        封装rospy.get_param,业务代码代码中调用该部分
        :param name: 参数名称
        :param default: 默认值
        :return: 参数值
        """
        if rospy.has_param(name):
            return rospy.get_param(name)
        else:
            rospy.logerr(" Parameter [%s] not found, defaulting to %.3f" % (name, default))
            return default
        
    # 读取状态机当前正在执行的状态
    # 如果返回NONE，表示状态机没有启动
    # 返回状态：TODO状态列表
    def get_active_state(self):
        """
        获取状态机的状态
        """
        if rospy.has_param("fsm_node/active_states"):
            state1 = self.get_param("fsm_node/active_states1","NONE")
            state2 = self.get_param("fsm_node/active_states2","NONE")
            state3 = self.get_param("fsm_node/active_states3","NONE")

            state = state1+state2+state3
            return state
        else:
            rospy.logerr(" fsm not start")


if __name__ == '__main__':
    # ros节点的初始化
    rospy.init_node('call_fsm')

    test = CallFSM()

    # 主循环
    # 1.调用测试接口服务
    # 2.读取参数
    while not rospy.is_shutdown():
        # 测试接口
        test.CallTestEmpty()
        # 读取参数
        test_car_state = test.get_param("test_car_state",0)
        rospy.loginfo("test_car_state is %d",test_car_state)
        rospy.sleep(1)