#!/usr/bin/python2.7
#
# Converts a given ROS bag file to a Daikon dtrace file
#
# TODO: needs a bit more documentation

__author__ = 'Afsoon Afzal'

import rosbag
import os
import sys

TYPES = {
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


# Reports an error to the stdout and exits with a given code
def error(msg, code=1):
    print("ERROR: {}".format(msg))
    exit(code)

# TODO: refactor
def parse_msg_file(filename, topic):
    variables = []
    with open(filename, 'r') as f:
        stack = []
        tabs = -1
        for l in f:
            if not l.strip():  # TODO: obtuse
                continue
            typ, name = l.replace('\t', '').replace('\n', '').split(' ')
            t = l.count('\t')
            if tabs >= t:
                var_type, var_name = stack.pop()
                if var_type in TYPES:
                    if '[]' in TYPES[var_type]:
                        var_name += '[..]'
                    prefix = '.'.join(i[1] for i in stack)
                    variables.append((
                                     topic + '.' + prefix + '.' + var_name if prefix else var_name,
                                     TYPES[var_type]))
                if t < tabs:
                    stack.pop()
            tabs = t
            stack.append((typ, name))
        if len(stack) > 0:
            var_type, var_name = stack.pop()
            if var_type in TYPES:
                if '[]' in TYPES[var_type]:
                    var_name += '[..]'
                prefix = '.'.join(i[1] for i in stack)
                variables.append((
                                 topic + '.' + prefix + '.' + var_name if prefix else var_name,
                                 TYPES[var_type]))
    return variables


def parse_msg(msg, msg_header, topic):
    variables = []
    stack = []
    tabs = -1
    for l in msg.splitlines():
        if not l.strip():  # TODO: obtuse
            continue
        t = l.count('\t')
        l = l.replace('\t', '').replace('\n', '')
        index = l.find(': ')
        name, value = l[:index], l[index + 2:]
        if tabs >= t:
            var_name, var_val = stack.pop()
            prefix = '.'.join(i[0] for i in stack)
            prefix = topic + '.' + prefix + '.' + var_name if prefix else var_name
            if prefix in msg_header:
                variables.append((prefix,
                                  var_val if msg_header[prefix] != TYPES[
                                      'string'] else '"' + var_val + '"'))
            elif prefix + '[..]' in msg_header:
                val_list = eval(var_val)
                if msg_header[prefix + '[..]'] == TYPES['string[]']:
                    var_val = '[' + ' '.join(
                        '"' + i + '"' for i in val_list) + ']'
                else:
                    var_val = '[' + ' '.join(i for i in val_list) + ']'
                variables.append((prefix + '[..]', var_val))
            if t < tabs:
                stack.pop()
        tabs = t
        stack.append((name, value))
    if len(stack) > 0:
        var_name, var_val = stack.pop()
        prefix = '.'.join(i[0] for i in stack)
        prefix = topic + '.' + prefix + '.' + var_name if prefix else var_name
        if prefix in msg_header:
            variables.append((prefix, var_val if msg_header[prefix] != TYPES[
                'string'] else '"' + var_val + '"'))
        elif prefix + '[..]' in msg_header:
            val_list = eval(var_val)
            if msg_header[prefix + '[..]'] == TYPES['string[]']:
                var_val = '[' + ' '.join('"' + i + '"' for i in val_list) + ']'
            else:
                var_val = '[' + ' '.join(i for i in val_list) + ']'
            variables.append((prefix + '[..]', var_val))
    return variables


def initialize_tables(bag, service_related, config='', important_topics=[]):
    # TODO read config
    msg_table = {}
    program_point_table = {}
    publishers, subscribers = None, None
    for t, message, time in bag.read_messages(topics=['/rec/arch_pub', ]):
        publishers = eval(str(message)[5:])
    for t, message, time in bag.read_messages(topics=['/rec/arch_sub', ]):
        subscribers = eval(str(message)[5:])
    for topic in important_topics:
        msg_table[topic] = []
        program_point_table[topic] = [topic, ]
        if topic in service_related:
            continue
        if publishers and subscribers:
            for pub in publishers[topic]:
                for key in subscribers.keys():
                    if pub in subscribers[key]:
                        program_point_table[topic].append(key)
                        msg_table[key] = []

    return msg_table, program_point_table


# What is the output of this function?
#
# TODO: the "topics" argument isn't used
# TODO: what are the contents of the message table?
# TODO: why do we accept "toPublish" and return it? Should we not create a new
#       buffer and return that?
def read_bag_file(topics, program_point_table, msg_table, bag_file, headers,
                  toPublish):
    bag = rosbag.Bag(bag_file)
    for topic, msg, time in bag.read_messages(topics=list(msg_table.keys())):
        if topic in msg_table:
            variables = parse_msg(str(msg).replace('  ', '\t'),
                                  dict(headers[topic]), topic)
            msg_table[topic] = variables
        if topic in program_point_table:
            s = '\n' + topic + ':::POINT\n'
            breaked = False
            for t in program_point_table[topic]:
                if not msg_table[t] and headers[t]:
                    breaked = True
                    break
                for n, v in msg_table[t]:
                    s += n + '\n' + v + '\n1\n'
            if not breaked:
                toPublish[topic].append(s)
            s = ''
    bag.close()
    return toPublish


def write_declaration_header(decl_file):
    decl_file.write('decl-version 2.0\nvar-comparability none\n')


def write_declaration(topic, variables, decl_file):
    decl_file.write('\nppt ' + topic + ':::POINT\n')
    decl_file.write('ppt-type point\n')
    for name, typ in variables:
        decl_file.write('variable ' + name + '\n')
        decl_file.write('  var-kind variable\n')
        decl_file.write('  dec-type ' + typ + '\n')
        decl_file.write('  rep-type ' + typ + '\n')


# TODO: Why are we piping to a temporary file? We can just use Popen to get the
# std. out of a command
def get_msg_header(msg_type):
    err = os.system('rosmsg show ' + msg_type + ' | unexpand -t 2 > rosmsg.txt')
    if err:
        print "ERROR: not able to get ros message for this type %s" % msg_type
    return 'rosmsg.txt'


# TODO: what data are we writing?
def write_file(trace_file, to_record):
    for topic in to_record.keys():
        for point in to_record[topic]:
            trace_file.write(point)


# TODO: what is a complementary service file? Explain.
def read_complementary_service_file(service_complement):
    service_related = []
    with open(service_complement, "r") as complement:
        tempServiceRelates = complement.readlines()
        for x in tempServiceRelates:
            if x.replace('\n', "") not in service_related:
                service_related.append(x.replace('\n', ""))
            service_related.append('/rec/arch_srvs')
    return service_related


def main():
    if len(sys.argv) == 1:
        error('Please provide a .bag file and a service_handler '
              'complementary file.\n '
              'Since rosbag does not record service calls, service_handler \n'
              'generates a separate log of the calls made in order to infer \n'
              'invariants. If service_handler generated a complementary file \n'
              'please provide such. Otherwise type -i next to the command to \n'
              'ignore this feature.\n\n'
              'Example: rosbagtotrace.py rosbag.bag servicecomplement.txt\n'
              '         rosbagtotrace.py rosbag.bag -i')
    if len(sys.argv) < 2:
        error('No .bag file provided')
    if len(sys.argv) < 3:
        error('Since rosbag does not record service calls, service_handler \n'
              'generates a separate log of the calls made in order to infer \n'
              'invariants. If service_handler generated a complementary file \n'
              'please provide such. Otherwise type -i next to the command to \n'
              'ignore this feature.')

    bag_file = sys.argv[1]
    service_complement = sys.argv[2]
    service_related = []

    # Read the service complement file, if one was provided
    # TODO: what is the format of these service complement files?
    if service_complement != '-i':
        service_related = read_complementary_service_file(service_complement)
    to_record = {}

    # TODO: setting up global variables
    topics = ['/rec/arch_pub', '/rec/arch_sub']
    for topic in topics:
        to_record[topic] = []
    for service in service_related:
        topics.append(service)
        to_record[service] = []

    print "\n Program Points: "
    for x in topics:
        print '    ' + x

    headers = {}
    bag = rosbag.Bag(bag_file)
    msg_table, program_point_table = initialize_tables(bag, service_related,
                                                       important_topics=topics)
    bag_topics = bag.get_type_and_topic_info()[1]
    decl_file = open(bag_file + '.dtrace', 'w')
    write_declaration_header(decl_file)
    for t in topics:
        vars = []
        for tt in program_point_table[t]:
            bag_t = bag_topics.get(tt)
            if not bag_t:
                print "ERROR: there is no such topic in this bag %s" % t
                continue
            msg_type = bag_t[0]
            file_name = get_msg_header(msg_type)
            variables = parse_msg_file(file_name, tt)
            headers[tt] = variables
            vars.extend(variables)
        write_declaration(t, vars, decl_file)
    bag.close()

    write_file(decl_file, read_bag_file(topics, program_point_table,
                                    msg_table, bag_file, headers, to_record))
    decl_file.close()


if __name__ == '__main__':
    main()
