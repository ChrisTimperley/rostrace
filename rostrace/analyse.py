#!/usr/bin/env python
import sys

import rosbag

def analyse_architecture(file_names):
    file_name = file_names[0]
    with rosbag.Bag(file_name) as bag:
        for msg in bag.read_messages(topics=['/rec/arch']):
            print(msg.message)


def analyse(file_names):
    analyse_architecture(file_names)


if __name__ == "__main__":
    file_names = sys.argv[1:]
    analyse(file_names)
