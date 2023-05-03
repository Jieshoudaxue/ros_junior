#! /usr/bin/env python3

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("sim_pub")
    pub = rospy.Publisher("chatter", String, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "hello ycao %s" %rospy.get_time()

        pub.publish(msg)

        rospy.loginfo(msg.data)

        rate.sleep()

if __name__ == "__main__":
    main()