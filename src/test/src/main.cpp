#include "ros/ros.h"
#include "dashboard/Request.h"
#include "std_msgs/Int32.h"
#include <string>
#include "uart/Datapack.h"

const std::string uart_send_topic = "uart_send", uart_recv_topic = "uart_recv";

void cb(const uart::Datapack& dp){
    ROS_INFO("got ret: cmd=%d, param=%s",dp.CMD_ID,dp.param.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node;

    auto subscriber = node.subscribe(uart_recv_topic,10,cb);

    auto publisher = node.advertise<uart::Datapack>(uart_send_topic,100);

    uart::Datapack dp;
    dp.param = "abc";
    dp.CMD_ID = 250;

    ros::Rate loop_rate(100);

    while(ros::ok())
	{   
        ros::spinOnce();
        dp.CMD_ID++;
        publisher.publish(dp);

		loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
	}

    return 0;
}