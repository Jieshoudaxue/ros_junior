#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "spawn_turtle");
    ros::NodeHandle nh;

    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");

    ros::service::waitForService("spawn");

    for (int i = 0; i < 10; i ++) {
        turtlesim::Spawn::Request req;
        turtlesim::Spawn::Response resp;

        req.x = 1 + i;
        req.y = 1 + i;
        req.theta = M_PI/2;
        req.name = "Leo" + std::to_string(i);

        bool ok = spawnClient.call(req, resp);
        if (ok) {
            ROS_INFO("spawned a turtle named %s", resp.name.c_str());
        } else {
            ROS_ERROR("Failed to spawn");
        }

    }


    return 0;
}