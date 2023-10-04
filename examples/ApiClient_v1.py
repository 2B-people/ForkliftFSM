#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty

class CallFSM():
    def __init__(self):
        # service的初始化
        self.call_fsm = rospy.ServiceProxy('fsm_node/test_empty', Empty)
        self.call_fsm.wait_for_service()
    
    # 使用ros的service接口调用状态机的测试接口
    # 这里用了一个不带参数的空请求进行测试
    def CallTestEmpty(self):
        """
        调用测试接口
        """
        try:
            response = self.call_fsm()
            rospy.loginfo("send req")
            return response
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed:")
            return None

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
            rospy.logwarn("Parameter [%s] not found, defaulting to %.3f" % (name, default))
            return default


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