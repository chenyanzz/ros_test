#include "ros/ros.h"
#include "serial/serial.h"
#include "datapack.h"
#include "uart/Datapack.h"

const std::string uart_send_topic = "uart_send", uart_recv_topic = "uart_recv";

ros::Subscriber sub;
ros::Publisher pub;

void datapack_send_callback(const uart::Datapack& dp){
    dp_send(dp.CMD_ID,dp.param.c_str());
}

void datapack_recv_callback(int32_t cmd_id, String param){
    uart::Datapack dp;
    dp.CMD_ID = cmd_id;
    dp.param = param;

    pub.publish(dp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial");

    ros::NodeHandle node;

    std::string serial_name = "/dev/ttyUSB0";
    int baud = 115200;

    if (argc > 1)
        serial_name = argv[1];

    serial::Serial ser(serial_name, 115200, serial::Timeout::simpleTimeout(1000));
    if (!ser.isOpen())
        ser.open();

    ROS_INFO("Serial \"%s\" with baud %d started.", serial_name.c_str(), baud);
    dp_init(&ser);
    dp_setDataCallback(datapack_recv_callback);

    sub = node.subscribe(uart_send_topic,100,datapack_send_callback);
    pub = node.advertise<uart::Datapack>(uart_recv_topic,100);

    while (ros::ok())
    {
        ros::spinOnce();
        if(ser.available()==0) continue;
        std::string s = ser.readline();
        dp_onRecv(s.c_str());

        ros::Rate(500).sleep();
    }

    return 0;
}