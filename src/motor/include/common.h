#ifndef COMMON_T
#define COMMON_T

typedef float fp32_t;
typedef short int16_t;
typedef unsigned short uint16_t;

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }



#endif // !COMMON_T