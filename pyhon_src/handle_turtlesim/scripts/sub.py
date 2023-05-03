#! /usr/bin/env python3

import rospy
from turtlesim.msg import Pose

def pose_cb(msg):
    rospy.loginfo("i received: [%s, %s], direction: %s" %(msg.x, msg.y, msg.theta));

def main():
    rospy.init_node("sub_pose")

    rospy.Subscriber("turtle1/pose", Pose, pose_cb)


    rospy.spin()





if __name__ == "__main__":
    main()