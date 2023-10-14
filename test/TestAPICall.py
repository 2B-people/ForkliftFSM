#!/usr/bin/env python
# -*- coding: utf-8 -*-

import readchar
import rospy
from APIClient_v1 import CallFSM

def test_fsm(fsm):
    """
    测试函数,根据键盘输入调用CallFSM对象的相应方法。
    """
    # 循环等待键盘输入
    print('Press "q" to quit.')
    print('Press "t" to call task interface.')
    print('Press "n" to call navigation interface.')
    print('Press "r" to call relocation interface.')
    print('Press "p" to call pickup interface.')
    print('Press "c" to call charge interface.')
    print('Press "s" to set charge point.')
    print('Press "g" to get current state.')
    print('Press "h" to call repair in.')
    print('Press "j" to call repair out.')
    
    while True:
        # 读取键盘输入
        key = readchar.readkey()

        # 根据输入执行相应的操作
        if key == 'q':
            print('The "q" key is pressed.')
            break
        elif key == 't':
            print('Calling task interface...')
            res=fsm.CallTask((1, 2), (3, 4))
            print(res)
            
        elif key == 'n':
            print('Calling navigation interface...')
            res=fsm.CallNav((5, 6))
            print(res)
        elif key == 'r':
            print('Calling relocation interface...')
            res=fsm.CallRelocation()
            print(res)
        elif key == 'p':
            print('Calling pickup interface...')
            res=fsm.CallPickup(1)
            print(res)
        elif key == 'c':
            print('Calling charge interface...')
            res=fsm.CallCharge()
            print(res)
        elif key == 's':
            print('Set Charge point (1, 2)...')
            res=fsm.Set_Charge_point((1, 2))
            print(res)
        elif key == 'g':
            state = fsm.get_active_state()
            print('Current state is {state}'.format(state=state))
        elif key == 'h':
            fsm.CallRepairIn()
            print('Calling repair interface...')
        elif key == 'j':
            fsm.CallRepairOut()
            print('Calling repair out...')

def main():
    # 创建CallFSM对象   
    fsm = CallFSM()

    # 初始化ROS节点
    rospy.init_node('test_call_fsm')
    rospy.loginfo("test_call_fsm is online")
    

    # 循环调用测试函数，直到用户按下“q”键退出程序
    test_fsm(fsm)

if __name__ == "__main__":
   main() 

