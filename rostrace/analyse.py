#!/usr/bin/env python
#
#
#
# Possible questions:
# - if X is present, what other nodes must be present?
# - if A is subscribe to T, B must publish to T {S(A,T) => P(B,T)}
#
#
# Do we want to think about Gazebo-based service calls?
import sys
import json
import rosbag

from pprint import pprint as pp

"""
Returns a sequence of architecture states encountered during a given run.

:param  file_name   the path to the ROS bag containing the run data

TODO: assumes the existence of the file
"""
def get_architecture_states(file_name):
    archs = []
    with rosbag.Bag(file_name) as bag:
        for msg in bag.read_messages(topics=['/rec/arch']):
            arch = json.loads(msg.message.data)

            # remove debugging-related topics and services
            del arch['publishers']['/rec/arch']
            del arch['publishers']['/rec/srvs']
            del arch['publishers']['/rosout']
            del arch['publishers']['/rosout_agg']

            del arch['subscribers']['/rec/arch']
            del arch['subscribers']['/rec/srvs']
            del arch['subscribers']['/rosout']
            del arch['subscribers']['/rosout_agg']

            for service in arch['services'].keys():
                if service.endswith('/get_loggers') or service.endswith('/set_logger_level'):
                    del arch['services'][service]

            archs.append(arch)

    return archs


def analyse_architecture(file_names):
    publishers = {}
    subscribers = {}
    services = {}

    # find the set of all publishers and subscribers to topics
    for file_name in file_names:
        arch_states = get_architecture_states(file_name)

        for arch in arch_states:
            for (topic, topic_publishers) in arch['publishers'].items():
                if not topic in publishers:
                    publishers[topic] = set()
                publishers[topic].update(topic_publishers)

            for (topic, topic_subscribers) in arch['subscribers'].items():
                if not topic in subscribers:
                    subscribers[topic] = set()
                subscribers[topic].update(topic_subscribers)

            for (service, servers) in arch['services'].items():
                if not service in services:
                    services[service] = set()
                services[service].update(servers)


    pp(publishers)
    

def analyse(file_names):
    analyse_architecture(file_names)


if __name__ == "__main__":
    file_names = sys.argv[1:]
    analyse(file_names)
