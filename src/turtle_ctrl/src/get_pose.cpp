#include "ros/ros.h"
#include "turtlesim/Pose.h"

void pose_callback(const turtlesim::Pose& pose){
    ROS_WARN("turtle{x=%f, y=%f, theta=%f, spd=%f, aspd=%f}",pose.x,pose.y,pose.theta,pose.linear_velocity,pose.angular_velocity);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_pose");
    ros::NodeHandle node;

    auto sub = node.subscribe("/turtle1/pose",10,pose_callback);

    ros::spin();
    
    return 0;
}