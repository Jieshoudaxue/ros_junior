#! /usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("pub_velocity")
    pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = random.random()
        msg.angular.z = random.random()

        pub.publish(msg)
        
        rospy.loginfo("sending rand velocity cmd: linear = %s, angular = %s" %(msg.linear.x, msg.angular.z))

        rate.sleep()

if __name__ == "__main__":
    main()