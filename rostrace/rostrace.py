#!/usr/bin/env python
import thread
import rospy

import service
import architecture
import bag

def trace():
    rospy.init_node('rostrace')

    # setup service re-direction
    # blocks until all services are redirected
    services = ['/hello_world']
    service.tap_services(services)

    # setup architecture monitoring
    thread.start_new_thread(architecture.record, ())

    # use semaphore to communicate status of architecture
    # log to bag file
    thread.start_new_thread(bag.record, ())

    rospy.spin()

if __name__ == "__main__":
    trace()
