#include "ros/ros.h"
#include "motor.h"
#include "motor/MotorInfo.h"
#include "motor/MotorCtrl.h"

const std::string motor_read_topic="motor_read", motor_write_topic="motor_write";

ros::Subscriber sub;
ros::Publisher pub;

void motorCtrl_callback(const motor::MotorCtrl& ctrl){
    for(int i=1;i<=8; i++)
        motor_setPower(i, ctrl.power[i-1]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor");
    ros::NodeHandle node;

    motors_init(node);
    sub = node.subscribe(motor_write_topic,100,motorCtrl_callback);
    pub = node.advertise<motor::MotorInfo>(motor_read_topic,100);

    motor::MotorInfo info;

    ros::Rate loop_rate(100);
    while(ros::ok()){
        for(int i=1;i<=8;i++){
            info.power[i-1] = motor_getPower(i);
            info.speed[i-1] = motor_getSpeed(i);
        }
        pub.publish(info);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }
    
    return 0;
}