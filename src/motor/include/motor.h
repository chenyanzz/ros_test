#pragma once

#include "ros/ros.h"

typedef unsigned int motor_id_t;

void motors_init(ros::NodeHandle &node);

//id: 1 ~ 8
//power: -10000 ~ +10000
void motor_setPower(motor_id_t id, int16_t power);

//id: 1 ~ 8
//return: -10000 ~ +10000
int16_t motor_getPower(motor_id_t id);

//id: 1 ~ 8
//return: RPM
int16_t motor_getSpeed(motor_id_t id);