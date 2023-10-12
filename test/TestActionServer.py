#!/usr/bin/env python

import rospy
import actionlib
from ForkliftFSM.msg import PurePursuitAction, PurePursuitResult, PurePursuitFeedback

def pure_pursuit_callback(goal):
    # Extract start and end points from goal
    start_xy = goal.start_xy
    end_xy = goal.end_xy

    # Initialize feedback and result
    feedback = PurePursuitFeedback()
    result = PurePursuitResult()

    # TODO: Implement Pure Pursuit algorithm
    rospy.sleep(2.0)

    # Set final position and orientation in result
    result.status = 1
    result.final_xy = [0.0, 0.0]
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
    
    rospy.loginfo("test_action_server is online")

    # Spin node
    rospy.spin()