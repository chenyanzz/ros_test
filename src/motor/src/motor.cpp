#include "ros/ros.h"
#include "can_msgs/Frame.h"
#include "common.h"
#include "motor.h"

static ros::Publisher publisher;
static ros::Subscriber subscriber;

static uint16_t speeds[8];//rpm
static int16_t powers[8];

static void canframe_callback(const can_msgs::Frame &frame)
{
    if(frame.id>=0x201 && frame.id <=0x208){
        int motor_id = frame.id - 0x201;
        speeds[motor_id] = (uint16_t)(frame.data[2]<<8 | frame.data[3]);
    }
}

static void sendData(){
   can_msgs::Frame frame;
   frame.dlc=8;
   frame.is_error=false;
   frame.is_extended=false;
   frame.is_rtr=false;
   
   frame.id=0x200;
   for(int i=0;i<4;i++){
       frame.data[2*i] = speeds[i]>>8;
       frame.data[2*i+1] = speeds[i];
   }
   publisher.publish(frame);

   frame.id = 0x1ff;
   for(int i=0;i<4;i++){
       frame.data[2*i] = speeds[i+4]>>8;
       frame.data[2*i+1] = speeds[i+4];
   }
   publisher.publish(frame);
}

void motors_init(ros::NodeHandle &node)
{
    publisher = node.advertise<can_msgs::Frame>("A",100);
    subscriber = node.subscribe("B",100,&canframe_callback);
}

void motor_setPower(unsigned int id, int16_t power)
{
    id--;
    assert (id<=8);
    LimitMax(power, 9999);
    powers[id] = power;
    sendData();
}

int16_t motor_getPower(motor_id_t id){
    id--;
    assert (id<=8);
    return powers[id];
}

//ABS of SPEED !!
uint16_t motor_getSpeed(motor_id_t id){
    id--;
    assert (id<=8);
    return speeds[id];
}