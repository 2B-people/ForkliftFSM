#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist

# 定义起始状态
class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_forward','go_hard'])
        self.cmd_vel_sub = rospy.Subscriber('/turtle1/cmd_vel', Twist, self.cmd_vel_callback)
        self.a = 2

    def cmd_vel_callback(self, data):
        # 处理接收到的速度命令
        self.current_cmd_vel = data
        self.a = 1

    def execute(self, userdata):
        rospy.loginfo("Starting the robot...")
        if  self.a == 1:
            return 'go_forward'
        elif self.a == 2:
            return 'go_hard'

# 定义前进状态
class ForwardState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])
        self.cmd_vel_sub = rospy.Subscriber('/turtle1/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, data):
        # 处理接收到的速度命令
        self.current_cmd_vel = data
        rospy.loginfo("Received cmd_vel: linear.x={data.linear.x}, angular.z={data.angular.z}")


    def execute(self, userdata):
        rospy.loginfo("Moving forward...")
        rospy.sleep(10)  # 模拟前进操作
        return 'stop'

# 定义停止状态
class StopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("Stopping the robot.")
        rospy.sleep(2)
        return 'done'

def main():
    rospy.init_node('simple_state_machine')

    # 创建状态机
    sm = smach.StateMachine(outcomes=['done'])

    # 添加状态到状态机
    with sm:
        smach.StateMachine.add('START', StartState(), transitions={'go_forward': 'FORWARD','go_hard':'START'})
        smach.StateMachine.add('FORWARD', ForwardState(), transitions={'stop': 'STOP'})
        smach.StateMachine.add('STOP', StopState(), transitions={'done': 'done'})

    # 创建状态机的执行器
    sis = smach_ros.IntrospectionServer('simple_state_machine', sm, '/SM_ROOT')
    sis.start()

    # 运行状态机
    outcome = sm.execute()

    # 停止状态机的可视化
    sis.stop()

if __name__ == '__main__':
    main()