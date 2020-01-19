#ifndef PID_H
#define PID_H

#include "common.h"

enum PID_MODE
{
  PID_POSITION = 0,
  PID_DELTA
};

typedef struct
{
  enum PID_MODE mode;
  fp32_t Kp;
  fp32_t Ki;
  fp32_t Kd;

  fp32_t max_out;
  fp32_t max_iout;

  fp32_t set;
  fp32_t fdb;

  fp32_t out;
  fp32_t Pout;
  fp32_t Iout;
  fp32_t Dout;
  fp32_t Dbuf[3];
  fp32_t error[3];

} pid_data_t;

extern void PID_Init(pid_data_t *pid, enum PID_MODE mode, fp32_t kp, fp32_t ki, fp32_t kd, fp32_t max_out, fp32_t max_iout);
extern fp32_t PID_Calc(pid_data_t *pid, fp32_t ref, fp32_t set);

#endif