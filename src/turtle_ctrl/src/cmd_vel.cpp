#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_ctl");
    ros::NodeHandle node;

    auto publisher = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);

    ros::Rate loop_rate(10);	//设置发送数据的频率为10Hz

    geometry_msgs::Twist t;
    t.angular.z = 0.2;
    t.linear.x = 0.5;

	while(ros::ok())
	{
		publisher.publish(t);
        ros::spinOnce();
		loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
	}
    
    return 0;
}