#include "ros/ros.h"
#include "dashboard/Request.h"
#include "std_msgs/Int32.h"
#include <string>

void cb(const std_msgs::Int32 str){
    ROS_WARN("got ret: %d",str.data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node;

    auto Pub = node.subscribe("dashboard_reply/num_test",10,cb);

    auto publisher = node.advertise<dashboard::Request>("dashboard/num",100);

    dashboard::Request req;
    req.action="set";
    req.editable = true;
    req.name="num_test";
    req.type="number";
    req.data="150|0|300|3";

    ros::Rate loop_rate(100);

    while(ros::ok())
	{       
        static int a=0;
        req.data=std::to_string(a)+"|0|1000|10";
        a+=123;
        if(a>=1000) a=0;
        publisher.publish(req);

		loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
        ros::spinOnce();
	}

    return 0;
}