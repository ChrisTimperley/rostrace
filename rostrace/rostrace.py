#!/usr/bin/env python
import thread
import rospy

import os
import subprocess
import signal

import service
import architecture

def trace():
    rospy.init_node('rostrace')
    try:
        # use semaphore to communicate status of architecture
        # log to bag file
        p_recorder = subprocess.Popen("rosbag record -a", preexec_fn=os.setsid, shell=True)
        print("Recording to rosbag...")
        thread.start_new_thread(p_recorder.communicate, ())

        # architecture monitoring
        thread.start_new_thread(architecture.record, ())

        # setup service re-direction
        # blocks until all services are redirected
        services = ['/hello_world']
        service.tap_services(services)

        # spin!
        rospy.spin()

    # safely stop recording
    finally:
        os.killpg(p_recorder.pid, signal.SIGINT)


def main():
    trace()
