#! /usr/bin/env python3

import rospy
import random
from std_msgs.msg import String
from beginner_tutorials.msg import Num

def talker():
    rospy.init_node("beg_talker", anonymous=True)
    
    std_pub = rospy.Publisher("chatter", String, queue_size=10)
    num_pub = rospy.Publisher("num", Num, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = String()

        hello_str = "hello ycao %s" %rospy.get_time()
        msg.data = hello_str
        std_pub.publish(msg)

        msg = Num()
        msg.num = random.randint(1, 100)

        num_pub.publish(msg)


        rospy.loginfo(hello_str)

        rate.sleep()


if __name__ == "__main__":
    talker()