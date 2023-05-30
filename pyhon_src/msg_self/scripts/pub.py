#! /usr/bin/env python3

import rospy
from msg_self.msg import Student

def main():
    rospy.init_node("msg_pub")

    pub = rospy.Publisher("student", Student, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        s1 = Student()
        s1.name = "jieshoudaxue"
        s1.age = 30

        pub.publish(s1)

        rospy.loginfo("send s1, name = %s, age = %d" %(s1.name, s1.age))
        rate.sleep()



if __name__ == "__main__":
    main()