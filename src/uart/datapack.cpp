#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "datapack.h"
#include "crc.h"
#include "ros/ros.h"
#include "serial/serial.h"

//a datapack is like: @ [crc] # [cmd_id] [param] $
#define DATA_BEGIN "@"
#define DATA_END "$"
#define DATA_SEPERATOR "#"
const int datapack_cntParts = 3;

static crc_t last_msg_crc = -1;

static dp_callback_t data_callback = (dp_callback_t)0;

serial::Serial *ser;

void dp_send(int32_t cmd_id, String param)
{
	static char str_back[100]; //用来crc的部分
	static char str[100];
	sprintf(str_back, DATA_SEPERATOR " %d %s " DATA_END, (int)cmd_id, param);
	crc_t crc8 = calc_crc(str_back, strlen(str_back));
	sprintf(str, DATA_BEGIN " %d %s\n", (int)crc8, str_back);
	for (int i = 0; i < 3; i++)
	{
		ser->write(std::string(str));
		ROS_INFO("sent datapck: \"%s\"",str);
		ser->flush();
		ros::Rate(1000).sleep();
	}
}

void dp_onRecv(String str)
{
	uint8_t crc;
	int32_t cmd_id;
	char param[100]; //注意多个参数不能以空格分隔
	{
		int _crc, _cmd_id;
		if (sscanf(str, DATA_BEGIN " %d " DATA_SEPERATOR " %d %s " DATA_END, &_crc, &_cmd_id, param) != datapack_cntParts)
			return; //参数数量不匹配
		crc = _crc;
		cmd_id = _cmd_id;
	}
	//check crc8
	const char *crc_begin = strstr(str, DATA_SEPERATOR);
	const char *crc_end = strstr(str, DATA_END);
	if (calc_crc(crc_begin, crc_end - crc_begin + 1) != crc)
		return;

	//判断是否是重复信息
	if (last_msg_crc == crc)
		return;
	last_msg_crc = crc;

	//处理函数
	if (data_callback)
	{
		data_callback(cmd_id, param);
	}
}

void dp_setDataCallback(dp_callback_t callback)
{
	data_callback = callback;
}

void dp_init(serial::Serial *_ser)
{
	ser = _ser;
}