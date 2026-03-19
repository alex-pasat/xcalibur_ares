#ifndef MOTOR_UTILS_H
#define MOTOR_UTILS_H

#include <stdint.h>

/* Includes ------------------------------------------------------------------*/

#define RPS_TO_RPM(rps) ((uint32_t)((rps) * 60.0f))
#define RPM_TO_RPS(rpm) ((float)(rpm) / 60.0f)

#define DEG_PER_REV 360.0f
#define RAD_PER_REV (2.0f * 3.14159265f)

#define DEG_TO_RAD(deg) ((deg) * RAD_PER_REV / DEG_PER_REV)
#define RAD_TO_DEG(rad) ((rad) * DEG_PER_REV / RAD_PER_REV)

typedef enum
{
    DIRECTION_CCW = 0,
    DIRECTION_CW = 1
} motor_direction_e;

uint16_t rps_to_rpm(float rps);



#endif /* MOTOR_UTILS_H */