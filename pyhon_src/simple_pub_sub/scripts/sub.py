#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

def msg_cb(msg):
    tmp_str = "%s: i received %s" %(rospy.get_caller_id(), msg.data)
    rospy.loginfo(tmp_str)

def main():
    rospy.init_node("sim_sub")

    rospy.Subscriber("chatter", String, msg_cb)

    rospy.spin()



if __name__ == "__main__":
    main()