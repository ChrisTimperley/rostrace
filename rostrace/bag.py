#!/usr/bin/env python
import service
import architecture
import os
import subprocess
import signal

"""
Records to a rosbag until an exception occurs
"""
def record():
    cmd = "rosbag record -a"
    p = None
    try:
        p = subprocess.Popen(cmd, preexec_fn=os.setsid, shell=True)
        p.communicate()
    except:
        if not p is None:
            os.killpg(p.pid, signal.SIGINT)

if __name__ == "__main__":
    record()
