#!/usr/bin/python
#
# This file is responsible for reading data from a given ROS bag file and
# transforming it into a set of program points, usable by Daikon
#
import sys
import rosbag
import yaml

from subprocess import Popen, PIPE
from pprint import pprint as pp

# Flattens a set of nested dictionaries into a single dictionary

# Writes to a dtrace file
def write_to_dtrace(data):
    pass

# Flattens the contents of a (decoded) message into a dictionary from flattened
# to values.
#def flatten_message_content(msg, prefixes=[]):
#    if not isinstance(msg, dict):
#        return {}
#    
#    out = {}
#    for (name, val) in msg.items():
#        if isinstance(val, dict):
#            out += flatten_message_content(msg, prefixes + [name])
#        elif isinstance(val, list):
#
#    return out

# Extracts all variable values from a given message into a dictionary of the
# form {name: value}, where names are flattened into a one-dimensional form
# (e.g. pose.position.x).
#
# Returns None if there was an error whilst reading the message.
def extract_vars_from_message(topic, msg):
    msg = str(msg)

    # A bug in the YAML output formatting of certain ROS messages can cause
    # YAML to fail. We throw these messages away.
    try:
        msg = yaml.load(msg)
    except yaml.scanner.ScannerError:
        return None
    
    return {}

# Given the name of a message type, this method returns a mapping between
# (flattened) field names and their corresponding types. Any properties which
# cannot be recorded by rostrace (i.e. non-primitive properties) are omitted.
def get_message_fields(msg):
    fields = {}
    cmd = ["rosmsg", "show", msg]
    out = ""

    p = Popen(cmd, stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()

    if p.returncode != 0:
        msg = 'Unexpected return code from `rosmsg show {}`: {} ({})'.format(msg, p.returncode, err)
        raise RuntimeError(msg)

    # Extract each of the fields from the std. out
    prefix = []
    for line in out.splitlines():
        line = line.rstrip()

        # Skip any empty lines
        if line.lstrip() == '':
            continue

        # Determine the prefix for the property
        depth = (len(line) - len(line.lstrip())) / 2
        prefix = prefix[:depth]

        # Get the fully qualified name and type for this property
        (typ, suffix) = line.lstrip().split(' ')
        name = '.'.join(prefix + [suffix])
        fields[name] = typ

        # Add to the prefix stack
        prefix.append(suffix) 

    return fields

def convert_bag_to_program_points(filename):
    bag = rosbag.Bag(filename)
    for entry in bag:
        var_vals = extract_vars_from_message(entry.topic, entry.message)

        sys.exit(0)

        # pp(var_vals)

def main(): 
    pp(get_message_fields("sensor_msgs/CameraInfo"))
    # convert_bag_to_program_points(sys.argv[1])

if __name__ == "__main__":
    main()