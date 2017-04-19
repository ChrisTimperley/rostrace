#!/usr/bin/python
#
# This file is responsible for reading data from a given ROS bag file and
# transforming it into a set of program points, usable by Daikon
#
import sys
import rosbag
import yaml
import re

from subprocess import Popen, PIPE
from pprint import pprint as pp

# A mapping from ROS types to Daikon types
TYPE_MAP = {
    'bool': 'boolean',
    'int8': 'int',
    'uint8': 'int',
    'int16': 'int',
    'uint16': 'int',
    'int32': 'int',
    'uint32': 'int',
    'int64': 'int',
    'uint64': 'int',
    'float32': 'double',
    'float64': 'double',
    'string': 'java.lang.String',
    'bool[]': 'boolean[]',
    'int8[]': 'int[]',
    'uint8[]': 'int[]',
    'int16[]': 'int[]',
    'uint16[]': 'int[]',
    'int32[]': 'int[]',
    'uint32[]': 'int[]',
    'int64[]': 'int[]',
    'uint64[]': 'int[]',
    'float32[]': 'double[]',
    'float64[]': 'double[]',
    'string[]': 'java.lang.String[]'
}

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

@TODO   map types to Daikon primitive types, excluding any incompatible types

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
    skip_till_depth = None
    for line in out.splitlines():
        line = line.rstrip()

        # Skip any empty lines
        if line.lstrip() == '':
            continue

        # Determine the prefix for the field
        depth = (len(line) - len(line.lstrip())) / 2
        prefix = prefix[:depth]

        # Get the name and type for this field
        (typ, suffix) = line.lstrip().split(' ')
        name = '.'.join(prefix + [suffix])
        prefix.append(suffix)

        # If the an ancestor of this field was an array type, skip this field
        if not skip_till_depth is None:
            if depth == skip_till_depth:
                skip_till_depth = None
            else:
                continue

        # Find the Daikon type for the ROS type
        # - record, if there is a mapping
        # - ignore all descendants, if there is no mapping and is array type
        # - otherwise, skip this field, but not necessarily its descendants
        typ = re.sub(r"\[\d+\]", '[]', typ)
        if typ in TYPE_MAP:
            fields[name] = TYPE_MAP[typ]
        elif '[]' in typ:
            skip_till_depth = depth

    return fields

def convert_bag_to_program_points(filename):
    with rosbag.Bag(filename) as bag:

        # Extract the set of topics represented within the bag
        # For now, this is the set of topics that we're interested in
        # TODO: filter topics according to blacklist / whitelist
        all_topics = set(m.topic for m in bag)
        # topics = all_topics
        interested_topics = set([
            "/camera/depth/camera_info",
            "/camera/depth/image_raw",
            "/camera/depth/points",
            "/camera/parameter_descriptions",
            "/camera/parameter_updates",
            "/camera/rgb/camera_info",
            "/clock",
            "/cmd_vel_mux/active",
            "/cmd_vel_mux/input/navi",
            "/cmd_vel_mux/input/safety_controller",
            "/cmd_vel_mux/input/switch",
            "/cmd_vel_mux/input/teleop",
            "/cmd_vel_mux/parameter_descriptions",
            "/cmd_vel_mux/parameter_updates",
            "/depthimage_to_laserscan/parameter_descriptions",
            "/depthimage_to_laserscan/parameter_updates",
            "/gazebo/link_states",
            "/gazebo/model_states",
            "/gazebo/parameter_descriptions",
            "/gazebo/parameter_updates",
            "/gazebo/set_link_state",
            "/gazebo/set_model_state",
            "/joint_states",
            "/laserscan_nodelet_manager/bond",
            "/mobile_base/commands/motor_power",
            "/mobile_base/commands/reset_odometry",
            "/mobile_base/commands/velocity",
            "/mobile_base/events/bumper",
            "/mobile_base/events/cliff",
            "/mobile_base/sensors/bumper_pointcloud",
            "/mobile_base/sensors/core",
            "/mobile_base/sensors/imu_data",
            "/mobile_base_nodelet_manager/bond",
            "/odom",
            "/scan",
            "/tf",
            "/tf_static"
        ])
        topics = all_topics.intersection(interested_topics)

        # Fetch the message formats used by each of those topics
        topic_formats = {t: get_message_format_for_topic(t) for t in topics}

        # Read all messages published to topics of interest
        for msg in bag.read_messages(topics=topics):
            msg_fmt = topic_formats[msg.topic]
            vrs = extract_vars_from_message(msg.topic, msg.message, msg_fmt)
            pp(vrs)

def main(): 
    pp(get_message_format("dynamic_reconfigure/ConfigDescription"))
    # convert_bag_to_program_points(sys.argv[1])

if __name__ == "__main__":
    main()
