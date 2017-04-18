#!/usr/bin/python3
import rospy
import rostopic

"""
This function is responsible for implementing a ROS node which periodically
publishes the state of the architecture to the "rec/architecture" topic.

Each message published to this topic describes the topics to which a given node
is currently publishing.
"""
def record_publishers():
    pub = rospy.Publisher('rec/architecture_publishers', String, queue_size=10)
    rospy.init_node('architecture_publishers')
    rate = rospy.Rate(10) # 10 Hz (TODO: excessive?)

    previous_state = {}

    while not rospy.is_shutdown():

        # determine the state of the architecture
        # TODO: aren't unpublished topics just as interesting, if not more so?
        topics = rospy.get_published_topics()

        current_state = {}
        for topic, typ in topics:
            info = rostopic.get_info_text(topic)
            (publishers, subscribers) = parse_rostopic_info(info)
            new_publish[topic] = set(publishers)

        if last_publish != new_publish:
            p = {}
            for topic in new_publish.keys():
                p[topic] = list(new_publish[topic])
            last_publish = new_publish
            pub.publish(str(p))

        previous_state = current_state
        rate.sleep()

"""
Returns a tuple of lists, listing the names of the subscribers and publishers
to a given topic, respectively.
"""
def get_topic_publishers_and_subscribers(topic):
    publishers = []
    subscribers = []

    info = rostopic.get_info_text(topic)
    info = [l.strip() for l in info.splitlines()]
    assert info[0].startswith('Type:')
    assert info[2].startswith('Publishers:')
    info = info[3:]

    # reads the name of a node from a given line
    get_name = lambda l: l.split(' ')[1]



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

    return (publishers, subscribers)

if __name__ == "__main__":