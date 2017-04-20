import service
import architecture
import os
import subprocess
import signal

"""
Records to a rosbag until an exception occurs
"""
def record():
    print("Recording to rosbag...")
    print("hmmm")
    cmd = "rosbag record -a"
    p = None
    try:
        p = subprocess.Popen(cmd, preexec_fn=os.setsid, shell=True)
        p.communicate()
    finally:
        #p.send_signal(signal.SIGINT)
        #os.kill(p.pid, signal.SIGINT)
        print("Killing PG: {}".format(os.getpgid(p.pid)))
        os.kill(p.pid, signal.SIGKILL)
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
