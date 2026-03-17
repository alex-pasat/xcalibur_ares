#ifndef MOTOR_UTILS_H
#define MOTOR_UTILS_H

#include <stdint.h>

/* Includes ------------------------------------------------------------------*/

typedef enum
{
    DIRECTION_CCW = 0,
    DIRECTION_CW = 1
} motor_direction_e;

// TODO
uint16_t rps_to_rpm(float rps);

#endif /* MOTOR_UTILS_H */