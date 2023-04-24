#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from beginner_tutorials.msg import Num


def std_cb(data):
    tmp_str = "%s : i received %s" %(rospy.get_caller_id(), data.data)
    rospy.loginfo(tmp_str)

def num_cb(data):
    tmp_str = "%s : i received %s" %(rospy.get_caller_id(), data.num)
    rospy.loginfo(tmp_str)




def listener():
    rospy.init_node("beg_listener", anonymous=True)

    rospy.Subscriber("chatter", String, std_cb)
    rospy.Subscriber("num", Num, num_cb)

    rospy.spin()








if __name__ == "__main__":
    listener()