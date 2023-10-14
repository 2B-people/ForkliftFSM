#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from ForkliftFSM.msg import PurePursuitAction, PurePursuitResult, PurePursuitFeedback
from std_msgs.msg import Bool

import readchar

def pure_pursuit_callback(goal):
    # Extract start and end points from goal
    start_xy = goal.start_xy
    end_xy = goal.end_xy

    # Initialize feedback and result
    feedback = PurePursuitFeedback()
    result = PurePursuitResult()

    # TODO: Implement Pure Pursuit algorithm
    rospy.sleep(1.0)
    feedback.status = 0.5
    feedback.cur_xy = (end_xy[0]/3, end_xy[1]/3)
    feedback.cur_theta = 1.0
    server.publish_feedback(feedback)
    rospy.sleep(1.0)
    feedback.status = 0.7
    feedback.cur_xy = (end_xy[0]/2, end_xy[1]/2)
    feedback.cur_theta = 0.0
    server.publish_feedback(feedback)
    rospy.sleep(5.0)

    # Set final position and orientation in result
    result.status = 1
    result.final_xy = end_xy
    result.final_theta = 0.0

    # Send final result to client
    rospy.loginfo('Returning result: {result}'.format(result=result))
    server.set_succeeded(result)

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('test_action_server')

    # Create action server
    server = actionlib.SimpleActionServer('pure_pursuit', PurePursuitAction, pure_pursuit_callback, False)
    server.start()

    # 创建名为“is_stop_obstacle”的发布者，用于发布Bool消息
    pub = rospy.Publisher('is_stop_obstacle', Bool, queue_size=10)
    
    rospy.loginfo("test_action_server is online")
    
    # 循环等待键盘输入
    while not rospy.is_shutdown():
        msg = Bool(False)
        # key = readchar.readkey()
        # if key == 's':
        #     msg = Bool(True)
        # if key == 'r':
        #     msg = Bool(False)
        # if key == 'c':
        #     break
        
        # 发布Bool消息
        pub.publish(msg)