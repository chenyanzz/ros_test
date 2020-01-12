#include "ros/ros.h"
#include "tf/tf.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_m");
    ros::NodeHandle node;

    auto publisher = node.advertise<geometry_msgs::PoseStamped>("PointM",1000);

    ros::Rate loop_rate(10);	//设置发送数据的频率为10Hz

	while(ros::ok())
	{
        tf::Stamped<tf::Pose> pose;
        pose.setOrigin(tf::Vector3(1,1,1));
        pose.setRotation(tf::Quaternion(0,0,0));
        pose.frame_id_ = "A";

		geometry_msgs::PoseStamped M;
        tf::poseStampedTFToMsg(pose,M);
        


		publisher.publish(M);
        ros::spinOnce();
		loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
	}
    
    return 0;
}