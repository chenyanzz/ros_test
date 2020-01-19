#ifndef CRC_H
#define CRC_H

#include "common.h"

typedef uint8_t crc_t;

//CRC8 now
crc_t calc_crc(const void *Buf, uint32_t Len);

#endif // !CRC_H