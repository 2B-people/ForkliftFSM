#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
from std_srvs.srv import Trigger   # 导入ROS服务消息类型
import smach_ros

def gripper_result_cb(request):
    gripper_request = Trigger().Request
    gripper_request.success = True
    rospy.loginfo(gripper_request.message)
    return gripper_request

def main():
    rospy.init_node('smach_example')
    rospy.Service("test",Trigger, gripper_result_cb)
    
    # 创建状态机
    sm = smach.StateMachine(['succeeded'])
    with sm:


        smach.StateMachine.add('TRIGGER_GRIPPER',
                           smach_ros.ServiceState('test_ser',
                                        Trigger),
                           transitions={'succeeded':'succeeded','aborted':'TRIGGER_GRIPPER','preempted':'TRIGGER_GRIPPER'})
        
    sis = smach_ros.IntrospectionServer('smach_example', sm, '/SM_ROOT')
    sis.start()

    # 运行状态机
    outcome = sm.execute()

    # 停止状态机的可视化
    sis.stop()

if __name__ == '__main__':
    main()