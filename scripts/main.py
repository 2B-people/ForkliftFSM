#!/usr/bin/env python
# -*- coding: utf-8 -*-

import ActionState 
import Service

import smach
import rospy

if __name__ == '__main__':
    rospy.init_node('simple_state_machine')

    # Create a SMACH state machine
    
