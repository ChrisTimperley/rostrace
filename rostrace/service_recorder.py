#!/usr/bin/env python
#
# Limitations:
# - doesn't work if another node is using a persistent connection
import rospy
import std_srvs.srv
import std_msgs.msg
import rosgraph
import rospy.core

from rospy.impl.tcpros_base import TCPROSTransport

"""
All (tapped) service calls are broadcast to the /rec/srvs topic in a JSON
format
"""
ServiceCallPublisher = rospy.Publisher('/rec/srvs', std_msgs.msg.String)

"""
Acts a proxy, forwarding a given service call onto its intended recepient,
whilst logging details of the service call to the appropriate topic
"""
def handle(proxy, req):
    print("[poison]")
    ret = proxy(req)
    print("[/poison]")
    return ret

"""
Installs a tap on a given service, causing all activity on that service to be
logged to the /rec/srvs topic.
"""
def tap_service(service_name):
    # TODO
    service_msg_format = std_srvs.srv.Empty

    # block until the service is available
    rospy.wait_for_service(service_name)

    # create a persistent proxy to that service
    # inject a persistent connection into the proxy, so that when we replace
    # the original service, we can still forward messages onto the old one
    proxy = rospy.ServiceProxy(service_name, service_msg_format, persistent=True)

    # TODO: listen for failures
    # http://docs.ros.org/jade/api/rospy/html/rospy.impl.tcpros_service-pysrc.html#ServiceProxy
    master = rosgraph.Master('/service_tap')
    service_uri = master.lookupService(proxy.resolved_name)
    (dest_addr, dest_port) = rospy.core.parse_rosrpc_uri(service_uri)
    proxy.transport = TCPROSTransport(proxy.protocol, proxy.resolved_name) 
    proxy.transport.buff_size = proxy.buff_size
    proxy.transport.connect(dest_addr, dest_port, service_uri) 

    # create a new, tapped service, with the same name
    rospy.Service(service_name, service_msg_format, (lambda r: handle(proxy, r)))

def tap_services(services):
    rospy.init_node('/service_tap')
    for service in services:
        tap_service(service)
    rospy.spin()

if __name__ == "__main__":
    tap_services(['hello_world'])
