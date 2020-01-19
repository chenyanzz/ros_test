#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include "serial/serial.h"
#include "common.h"

typedef enum {
    CMD_2333 = 0
} CMD_ID;

typedef void (*dp_callback_t)(CMD_ID cmd_id, String param);

void dp_send(CMD_ID cmd_id, String param);//blocking
void dp_onRecv(String str);
void dp_setDataCallback(dp_callback_t cb);
void dp_init(serial::Serial* _ser);

#endif // !FRAMEWORK_H