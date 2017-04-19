#!/usr/bin/env python

import rospy
import rostopic
# from recorder.msg import Publishers
from std_msgs.msg import String


# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=False)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()


def arch_publisher():
    pub = rospy.Publisher('rec/arch_pub', String, queue_size=10)
    rospy.init_node('arch_pub', anonymous=False)
    rate = rospy.Rate(10)
    last_publish = {}
    while not rospy.is_shutdown():
        topics = rospy.get_published_topics()
        new_publish = {}
        for topic, typ in topics:
            info = rostopic.get_info_text(topic)
            publishers = parse_info(info)
            new_publish[topic] = set(publishers)
        if last_publish != new_publish:
            p = {}
            for topic in new_publish.keys():
                p[topic] = list(new_publish[topic])
            last_publish = new_publish
            pub.publish(str(p))
        rate.sleep()


def parse_info(info):
    reached_pub = False
    publishers = []
    for l in info.splitlines():
        if reached_pub and (not l or l.startswith('Subscribers:')):
            break
        elif reached_pub:
            parts = l.split(' ')
            if len(parts) < 3:
                rospy.loginfo("Something is wrong here!")
                continue
            publishers.append(parts[2])
        elif l.startswith('Publishers:'):
            reached_pub = True
    return publishers

if __name__ == '__main__':
    try:
        arch_publisher()
    except rospy.ROSInterruptException:
        pass

