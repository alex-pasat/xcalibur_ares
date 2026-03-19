#ifndef MOTOR_UTILS_H
#define MOTOR_UTILS_H

#include <stdint.h>

/* Includes ------------------------------------------------------------------*/

#define RPS_TO_RPM(rps) ((uint32_t)((rps) * 60.0f))
#define RPM_TO_RPS(rpm) ((float)(rpm) / 60.0f)

typedef enum
{
    DIRECTION_CCW = 0,
    DIRECTION_CW = 1
} motor_direction_e;

uint16_t rps_to_rpm(float rps);



#endif /* MOTOR_UTILS_H */