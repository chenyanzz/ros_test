#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotator");
    ros::NodeHandle node;
    
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(1,-2,0));
    transform.setRotation(tf::createQuaternionFromRPY(-90,0,45));

    tf::TransformBroadcaster broadcaster;

    ros::Rate loop_rate(10);	//设置发送数据的频率为10Hz
    
    while(ros::ok())
	{
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "A", "B"));
        //broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "B", "A"));
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
