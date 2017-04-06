__author__ = 'Afsoon Afzal'

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


def parse_msg_file(filename, topic):
    variables = []
    with open(filename, 'r') as f:
        stack = []
        tabs = -1
        for l in f:
            if not l.strip():
                continue
            typ, name = l.replace('\t', '').replace('\n', '').split(' ')
            t = l.count('\t')
            if tabs >= t:
                var_type, var_name = stack.pop()
                if var_type in TYPES:
                    if '[]' in TYPES[var_type]:
                        var_name += '[..]'
                    prefix = '.'.join(i[1] for i in stack)
                    variables.append((topic + '.' + prefix+'.'+var_name if prefix else var_name, TYPES[var_type]))
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
                variables.append((topic + '.' + prefix+'.'+var_name if prefix else var_name, TYPES[var_type]))
    return variables


def parse_msg(msg, msg_header, topic):
    variables = []
    stack = []
    tabs = -1
    for l in msg.splitlines():
        if not l.strip():
            continue
        t = l.count('\t')
        l = l.replace('\t', '').replace('\n', '')
        index = l.find(': ')
        name, value = l[:index], l[index+2:]
        if tabs >= t:
            var_name, var_val = stack.pop()
            prefix = '.'.join(i[0] for i in stack)
            prefix = topic + '.' + prefix+'.'+var_name if prefix else var_name
            if prefix in msg_header:
                variables.append((prefix, var_val if msg_header[prefix] != TYPES['string'] else '"'+var_val+'"'))
            elif prefix + '[..]' in msg_header:
                val_list = eval(var_val)
                if msg_header[prefix + '[..]'] == TYPES['string[]']:
                    var_val = '[' + ' '.join('"'+i+'"' for i in val_list) + ']'
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
        prefix = topic + '.' + prefix+'.'+var_name if prefix else var_name
        if prefix in msg_header:
            variables.append((prefix, var_val if msg_header[prefix] != TYPES['string'] else '"'+var_val+'"'))
        elif prefix + '[..]' in msg_header:
            val_list = eval(var_val)
            if msg_header[prefix + '[..]'] == TYPES['string[]']:
                var_val = '[' + ' '.join('"'+i+'"' for i in val_list) + ']'
            else:
                var_val = '[' + ' '.join(i for i in val_list) + ']'
            variables.append((prefix + '[..]', var_val))
    return variables
