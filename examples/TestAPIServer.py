#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty, EmptyResponse

def empty_service_callback(request):
    # 在这里执行你的处理逻辑，因为这是一个空请求，所以没有额外的数据需要处理。
    # 你可以在这里执行任何你想要的操作。

    rospy.loginfo("Received an empty request. Performing some action.")
    
    # 返回一个空的响应
    return EmptyResponse()

def empty_service_server():
    rospy.init_node('empty_service_server')
    service = rospy.Service('fsm_node/test_empty', Empty, empty_service_callback)
    rospy.loginfo("Empty service server is ready.")

    rospy.set_param("test_car_state",10)
    
    rospy.spin()

if __name__ == "__main__":
    empty_service_server()