#include "ros/ros.h"
#include "serial/serial.h"
#include "gyro/Gyro.h"

typedef unsigned char byte;
byte datapack[11];
float acc[3], aspeed[3], angle[3];

bool readDatapack(serial::Serial &ser)
{
    ser.flush();
    //wait for a start sign
    while (ser.read()[0] != 0x55)
        ;

    //receive it
    datapack[0] = 0x55;
    for (int i = 1; i <= 10; i++)
    {
        datapack[i] = ser.read()[0];
    }

    //checksum
    int sum = 0;
    for (int i = 0; i <= 9; i++)
        sum += datapack[i];
    if ((byte)sum != datapack[10])
        return false;

    //parse
    switch (datapack[1])
    {
    case 0x51:
        acc[0] = (short(datapack[3] << 8 | datapack[2])) / 32768.0 * 16;
        acc[1] = (short(datapack[5] << 8 | datapack[4])) / 32768.0 * 16;
        acc[2] = (short(datapack[7] << 8 | datapack[6])) / 32768.0 * 16;
        break;

    case 0x52:
        aspeed[0] = (short(datapack[3] << 8 | datapack[2])) / 32768.0 * 2000;
        aspeed[2] = (short(datapack[7] << 8 | datapack[6])) / 32768.0 * 2000;
        aspeed[1] = (short(datapack[5] << 8 | datapack[4])) / 32768.0 * 2000;
        break;

    case 0x53:
        angle[0] = (short(datapack[3] << 8 | datapack[2])) / 32768.0 * 180;
        angle[1] = (short(datapack[5] << 8 | datapack[4])) / 32768.0 * 180;
        angle[2] = (short(datapack[7] << 8 | datapack[6])) / 32768.0 * 180;
        break;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uart_gyro");

    ros::NodeHandle node;

    std::string serial_name = "/dev/ttyUSB0", topic_name = "gyro";
    int baud = 115200;

    if (argc > 1)
        serial_name = argv[1];

    serial::Serial ser(serial_name, 115200, serial::Timeout::simpleTimeout(1000));
    if (!ser.isOpen())
        ser.open();

    ROS_INFO("Serial \"%s\" with baud %d started.", serial_name.c_str(), baud);

    ros::Publisher pub = node.advertise<gyro::Gyro>(topic_name, 100);
    gyro::Gyro gyro_data;
    while (ros::ok())
    {
        if (readDatapack(ser))
            ROS_INFO("angle x:%.2f y:%.2f z:%.2f", angle[0], angle[1], angle[2]);

        for (int i = 0; i < 3; i++)
        {
            gyro_data.acc[i] = acc[i];
            gyro_data.angle[i] = angle[i];
            gyro_data.aspeed[i] = aspeed[i];
        }
        pub.publish(gyro_data);
    }

    return 0;
}