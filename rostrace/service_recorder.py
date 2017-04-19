#!/usr/bin/env python
#
# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
import rospy

"""
Acts a proxy, forwarding a given service call onto its intended recepient,
whilst logging details of the service call to the appropriate topic
"""
def forward(forward_to, srv_format, request):
    # wait for the service to become available
    rospy.wait_for_service(forward_to)
    try:

        # TODO: log the service call

        # make the call
        proxy = rospy.ServiceProxy(forward_to, srv_format)
        response = proxy(request)

        # TODO: log the response (and time taken)

        # return the response
        return response

    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))

def server():


if __name__ == "__main__":
    server()
