#!/usr/bin/env python
import rospy
import std_srvs.srv

def handle(req):
    if req.data:
        print("Hello, world!")
    else:
        print("Goodbye, cruel world!")
    return std_srvs.srv.SetBoolResponse(True, "can haz response?")

def server():
    rospy.init_node('hello_world_server')
    rospy.Service('hello_world', std_srvs.srv.SetBool, handle)
    rospy.spin()

if __name__ == "__main__":
    server()
