#! /usr/bin/env python3

import rospy
from msg_self.msg import Student


def stu_cb(stu):
    rospy.loginfo("%s: i received s1, name = %s, age = %d" %(rospy.get_caller_id(), stu.name, stu.age))

def main():
    rospy.init_node("msg_sub")
    rospy.Subscriber("student", Student, stu_cb)

    rospy.spin()


if __name__ == "__main__":
    main()