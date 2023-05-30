#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse

g_forward = True

def toggleForward(req):
    global g_forward
    g_forward = not g_forward

    fwd = "forward" if g_forward else "rotate"

    rospy.loginfo("now receiveing: %s" %fwd)

    return EmptyResponse()

def main():
    global g_forward

    rospy.init_node("toggle_server")

    srv = rospy.Service("toggle_forward", Empty, toggleForward)

    pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 1.0 if g_forward else 0.0
        msg.angular.z = 0.0 if g_forward else 1.0

        pub.publish(msg)

        rate.sleep()



    rospy.spin()

if __name__ == "__main__":
    main()