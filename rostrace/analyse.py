#!/usr/bin/env python
import sys
import json
import rosbag

from pprint import pprint as pp

def analyse_architecture(file_names):
    file_name = file_names[0]
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

            pp(arch)


def analyse(file_names):
    analyse_architecture(file_names)


if __name__ == "__main__":
    file_names = sys.argv[1:]
    analyse(file_names)
