#!/usr/bin/env python
#
# See; http://docs.ros.org/kinetic/api/rosgraph/html/rosgraph.masterapi.Master-class.html#registerService
import rospy
import std_msgs.msg
import json
import rosgraph

from pprint import pprint as pp

from subprocess import Popen
from os import devnull

"""
This function is responsible for implementing a ROS node which periodically
checks the state of the architecture, and if it has changed since the last
iteration, publishes that state to /rec/arch. The architecture state is
published as a JSON document.
"""
def record():
    master = rosgraph.Master("/rostrace")
    pub = rospy.Publisher('rec/arch', std_msgs.msg.String, queue_size=5)
    rate = rospy.Rate(0.2)

    previous_state = {}
    while not rospy.is_shutdown():
        (pubs, subs, servs) = master.getSystemState()
        current_state = {
            'publishers': {t: nodes for (t, nodes) in pubs},
            'subscribers': {t: nodes for (t, nodes) in subs},
            'services': {s: nodes for (s, nodes) in servs}
        }

        if previous_state != current_state:
            pub.publish(json.dumps(current_state))
            previous_state = current_state

        rate.sleep()

if __name__ == "__main__":
    # register this node as /rostrace
    rospy.init_node('rostrace')

    # start recording publishers (in another thread)
    record()

    # start recording ROS bag
    # TODO: for now, we record ALL topics
    #       in the future, we should record specific topics
#    cmd = "rosbag record -a"
#    with Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE, preexec_fn=preexecute, cwd=sandboxd) as p:
#
#        # TODO: listen out for bad return codes and errors
#        try:
#            p.communicate(timeout=tlim)
#
#        # TODO: be specific about what sort of exceptions we want to stop
#        # recording
#        except:
#            os.killpg(p.pid, signal.SIGKILL)
