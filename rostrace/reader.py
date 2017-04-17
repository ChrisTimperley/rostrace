#!/usr/bin/python
#
# This file is responsible for reading data from a given ROS bag file and
# transforming it into a set of program points, usable by Daikon
#
# - header is only included in some messages
# - messages are encoded in YAML

import sys
import rosbag
import yaml

from pprint import pprint as pp

# Writes to a dtrace file
def write_to_dtrace(data):
    pass

# Extracts all variable values from a given message
# Returns a dictionary the form {name: value}, where names are flattened
# into a one-dimensional form (e.g. pose.position.x).
#
# Returns None if there was an error whilst reading the message.
def extract_variable_values_from_message(topic, msg):
    msg = str(msg)

    # A bug in the YAML output formatting of certain ROS messages can cause
    # YAML to fail. We throw these messages away.
    try:
        msg = yaml.load(msg)
    except yaml.scanner.ScannerError:
        return None

    return {}

def convert_bag_to_program_points(filename):
    bag = rosbag.Bag(filename)
    for entry in bag:
        # entry.topic
        # entry.message

        var_vals = extract_variable_values_from_message(entry.topic, entry.message)

        # pp(var_vals)

def main(): 
    convert_bag_to_program_points(sys.argv[1])

if __name__ == "__main__":
    main()
