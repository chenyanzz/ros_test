#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

geometry_msgs::PoseStamped N;

void pointm_callback(const geometry_msgs::PoseStamped& msg){
    static tf::TransformListener listenner;
    try{
        listenner.transformPose("B",msg,N); 
    }catch(std::runtime_error e){
        ROS_INFO(e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_n");
    ros::NodeHandle node;

    auto suscriber = node.subscribe("PointM",1000,pointm_callback);
    auto publisher = node.advertise<geometry_msgs::PoseStamped>("PointN",1000);

    ros::Rate loop_rate(10);	//设置发送数据的频率为10Hz
    
    while(ros::ok())
	{
		publisher.publish(N);
        ros::spinOnce();
		loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
	}

    return 0;
}