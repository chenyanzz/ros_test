#include "ros/ros.h"
#include "motor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor");
    ros::NodeHandle node;

    motors_init(node);
    
    return 0;
}