#!/usr/bin/env python
import rospy
import std_srvs.srv

def handle(req):
    print("Hello, world!")
    return std_srvs.srv.EmptyResponse()

def server():
    rospy.init_node('hello_world_server')
    rospy.Service('hello_world', std_srvs.srv.Empty, handle)
    rospy.spin()

if __name__ == "__main__":
    server()
