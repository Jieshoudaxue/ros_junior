#include <ros/ros.h>
#include <msg_self/Student.h>

void stu_cb(const msg_self::Student::ConstPtr &msg) {
    ROS_INFO("i received: %s, %u", msg->name.c_str(), msg->age);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "msg_sub");
    ros::NodeHandle nh;

    ros::Subscriber num_sub = nh.subscribe("student", 1000, &stu_cb);

    ros::spin();
    return 0;
}