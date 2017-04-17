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

# Helper function for recursively finding the value of an inner dictionary
# entry
def fetch_at_location(dct, loc):
    key = loc[0]
    val = dct[key]
    loc = loc[1:]

    if loc == []:
        return nxt_val
    else:
        return fetch_at_location(val, loc)

"""
Extracts all variable values from a given message into a dictionary of the
form {name: value}, where names are flattened into a one-dimensional form
(e.g. pose.position.x).

Returns None if there was an error whilst reading the message.

@param  topic       the name of the topic the message was published to
@param  msg         the contents of the message, as provided by the ROS Bag API
@param  msg_format  the message format used by this topic
"""
def extract_vars_from_message(topic, msg, msg_format):
    msg = str(msg)

    # A bug in the YAML output formatting of certain ROS messages can cause
    # YAML to fail. We throw these messages away.
    try:
        msg = yaml.load(msg)
    except yaml.scanner.ScannerError:
        return None

    # Extract the message contents into a flat dictionary
    contents = {}
    for param in msg_format:
        location = param.split('.')
        param = '{}.{}'.format(topic, param)
        contents[param] = fetch_at_location(msg, location)

    return contents

"""
Returns details of the message format used by a particular topic

@see    get_message_format
"""
def get_message_format_for_topic(topic):

    # Get the name of the message format used by the topic
    cmd = ["rostopic", "type", topic]
    p = Popen(cmd, stdout=PIPE, stderr=PIPE)
    out, err = p.communicate()

    if p.returncode != 0:
        msg = 'Failed to get message format for topic ({}): {}'.format(topic, err)
        raise RuntimeError(msg)

    msg_format = out.strip()
    return get_message_format(msg_format)

"""
For a given message type, this method returns a mapping between (flattened)
field names and their corresponding types. Any properties which cannot be
recorded by rostrace (i.e. non-primitive properties) are omitted.

@param  msg the name of the message type
"""
def get_message_format(msg):
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
    with rosbag.Bag(filename) as bag:

        # Extract the set of topics represented within the bag
        # For now, this is the set of topics that we're interested in
        # TODO: filter topics according to blacklist / whitelist
        all_topics = set(m.topic for m in bag)
        topics = all_topics

        # Fetch the message formats used by each of those topics
        topic_formats = {t: get_message_format_for_topic(t) for t in topics}
        pp(topic_formats)

        #for entry in bag:
        #    var_vals = extract_vars_from_message(entry.topic, entry.message)
        #
        #    sys.exit(0)

        # pp(var_vals)

def main(): 
    convert_bag_to_program_points(sys.argv[1])

if __name__ == "__main__":
    main()
