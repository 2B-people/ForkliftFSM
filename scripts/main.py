#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading

import smach
from smach import StateMachine, Concurrence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer,set_preempt_handler, MonitorState

from ActionState import PurePursuitstate
from APIService import serviceServer


class IDLE_state(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded', 'failed', 'preempted'],
					   output_keys=['start_x', 'start_y'])

	def execute(self,userdata):
		rospy.loginfo('Executing state 1')
		rospy.sleep(1)
		rospy.set_param('start_x',1)
		rospy.set_param('start_y',1)
		userdata.start_x = 1
		userdata.start_y = 1
		return 'succeeded'

def main():
	rospy.init_node("Forkift_fsm_node")
	rospy.loginfo("Forkift_fsm_node started")

	# Create a service server
	srv = serviceServer()

	# Create a SMACH state machine
	sm_root = StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
	 
	with sm_root:
		StateMachine.add('IDLE', IDLE_state(),
						 transitions={'succeeded':'nav_cc'})
		StateMachine.add('nav_cc', PurePursuitstate(),
						 transitions={'succeeded':'IDLE'})
	

	# Attach a SMACH introspection server
	sis = IntrospectionServer('smach_usecase_01', sm_root, '/USE_CASE')
	sis.start()
	
	# Set preempt handler
	set_preempt_handler(sm_root)

	# Execute SMACH tree in a separate thread so that we can ctrl-c the script
	smach_thread = threading.Thread(target = sm_root.execute)
	smach_thread.start()

	rate = rospy.Rate(10) # 10hz
	# Signal ROS shutdown (kill threads in background)
	while not rospy.is_shutdown():
		active_states = sm_root.get_active_states()
		rospy.set_param('active_states',active_states)
		rate.sleep()

	sis.stop()
		




if __name__ == '__main__':
	main()

