#!/usr/bin/env python
#
# TODO: use ROS's logging facilities
#
import thread
import rospy
import time

import os
import subprocess
import signal

import service
import architecture

def trace():
    rospy.init_node('rostrace')

    services = ['/hello_world']

    try:
        # use semaphore to communicate status of architecture
        # log to bag file
        p_recorder = subprocess.Popen("rosbag record -a", preexec_fn=os.setsid, shell=True)
        print("Recording to rosbag...")
        thread.start_new_thread(p_recorder.communicate, ())
        time.sleep(2)

        # architecture monitoring
        thread.start_new_thread(architecture.record, ())

        # setup service re-direction
        # blocks until all services are redirected
        service.tap_services(services)

        # spin!
        rospy.spin()

    # safely stop recording and restore original services
    finally:
        os.killpg(p_recorder.pid, signal.SIGINT)
        service.restore_services(services)


def main():
    trace()
