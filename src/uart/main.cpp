#include "ros/ros.h"
#include "serial/serial.h"
#include "datapack.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial");

    ros::NodeHandle node;

    std::string serial_name = "/dev/ttyUSB0", topic_name = "gyro";
    int baud = 115200;

    if (argc > 1)
        serial_name = argv[1];

    serial::Serial ser(serial_name, 115200, serial::Timeout::simpleTimeout(1000));
    if (!ser.isOpen())
        ser.open();

    ROS_INFO("Serial \"%s\" with baud %d started.", serial_name.c_str(), baud);
    dp_init(&ser);

    while (ros::ok())
    {
        char c = getchar();
        if(c=='a')
            dp_send(CMD_2333,"1");
        else 
            dp_send(CMD_2333,"0");

        ros::Rate(10).sleep();
    }

    return 0;
}