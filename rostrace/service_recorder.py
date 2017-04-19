#!/usr/bin/env python
#
# TODO: need to get the appropriate message format for the service
#       - we could use some catkin and ROS to find it and dynamically
#           import it
#
# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
import rospy
import std_srvs.srv

"""
Acts a proxy, forwarding a given service call onto its intended recepient,
whilst logging details of the service call to the appropriate topic
"""
#def forward(forward_to, srv_format, request):
    # wait for the service to become available
#    rospy.wait_for_service(forward_to)
#    try:
#        # TODO: log the service call
#
#        # make the call
#        proxy = rospy.ServiceProxy(forward_to, srv_format)
#        response = proxy(request)
#
#        # TODO: log the response (and time taken)
#
#        # return the response
#        return response
#
#    except rospy.ServiceException, e:
#        print("Service call failed: {}".format(e))

def handle(proxy, req):
    print("[poison]")
    ret = proxy(req)
    print("[/poison]")
    return ret

def server():
    rospy.init_node('service_tap')

    # create a proxy to the original service
    rospy.wait_for_service('hello_world')
    proxy = rospy.ServiceProxy('hello_world', std_srvs.srv.Empty, persistent=True)

    # let's just try this?
    proxy()

    # create a new service
    rospy.Service('hello_world', std_srvs.srv.Empty, (lambda r: handle(proxy, r)))
    rospy.spin()

if __name__ == "__main__":
    server()
