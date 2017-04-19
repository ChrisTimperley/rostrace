#!/usr/bin/env python
#
# Limitations:
# - doesn't work if another node is using a persistent connection
# - all names MUST be fully qualified, else rosservice will fail
#
# http://docs.ros.org/diamondback/api/rosservice/html/index.html

import inspect
import rospy
import std_srvs.srv
import std_msgs.msg
import rosgraph
import rosservice
import rospy.core
import json

from pprint import pprint as pp

from rospy.impl.tcpros_base import TCPROSTransport

# we use the most accurate timer available to the system
from timeit import default_timer as timer

"""
All (tapped) service calls are broadcast to the /rec/srvs topic in a JSON
format. The +queue_size+ parameter creates an asynchronous publisher, which
is better suited to our needs (higher throughput)
"""
serviceCallPublisher = rospy.Publisher('rec/srvs', std_msgs.msg.String, queue_size=10)

"""
Acts a proxy, forwarding a given service call onto its intended recepient,
whilst logging details of the service call to the appropriate topic
"""
def handle(server, service_name, proxy, req):
    time_start = timer()
    client = req._connection_header['callerid']

    # generate a JSON-encodable description of the parameters for this request
    # TODO: will fail with complex, embedded objects
    params = {p: getattr(req, p) for p in req.__slots__}

    # send the request and wait for a response
    ret = proxy(req)

    # generate a JSON-encodable description of the response
    # TODO: may fail for more complex responses
    response = {p: getattr(ret, p) for p in ret.__slots__}

    # determine the response time
    time_end = timer()
    time_duration = time_end - time_start

    # log the service call
    log = {
        'service': service_name,
        'server': server,
        'client': client,
        'time_start': time_start,
        'time_end': time_end,
        'time_duration': time_duration,
        'params': params,
        'response': response
    }
    serviceCallPublisher.publish(json.dumps(log))

    return ret

"""
Installs a tap on a given service, causing all activity on that service to be
logged to the /rec/srvs topic.
"""
def tap_service(service_name):
    # block until the service is available
    rospy.wait_for_service(service_name)

    # determine which node provides the given service
    server = rosservice.get_service_node(service_name)
    assert not server is None

    # get the class used by this service
    service_cls = rosservice.get_service_class_by_name(service_name)

    # create a persistent proxy to that service
    # inject a persistent connection into the proxy, so that when we replace
    # the original service, we can still forward messages onto the old one
    proxy = rospy.ServiceProxy(service_name, service_cls, persistent=True)

    # TODO: listen for failures
    # http://docs.ros.org/jade/api/rospy/html/rospy.impl.tcpros_service-pysrc.html#ServiceProxy
    master = rosgraph.Master('service_tap')
    service_uri = master.lookupService(proxy.resolved_name)
    (dest_addr, dest_port) = rospy.core.parse_rosrpc_uri(service_uri)
    proxy.transport = TCPROSTransport(proxy.protocol, proxy.resolved_name) 
    proxy.transport.buff_size = proxy.buff_size
    proxy.transport.connect(dest_addr, dest_port, service_uri) 

    # create a new, tapped service, with the same name
    tap = lambda r: handle(server, service_name, proxy, r)
    rospy.Service(service_name, service_cls, tap)

def tap_services(services):
    rospy.init_node('service_tap')
    for service in services:
        tap_service(service)
    rospy.spin()

if __name__ == "__main__":
    tap_services(['/hello_world'])
