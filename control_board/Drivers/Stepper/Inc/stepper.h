#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include "stm32g4xx.h"

#define NUM_STEPPERS 2
#define STEPPER_MICROSTEP_ENABLED 0

typedef enum
{
    CCW,
    CW
} stepper_direction_e;

typedef enum
{
    STEPPER_OFF = 0,
    STEPPER_ON = 1,
} stepper_status_e;

#if STEPPER_MICROSTEP_ENABLED
typedef enum {
    MS_FULL  = 0,
    MS_HALF  = 1,
    MS_1_4   = 2,
    MS_1_8   = 3,
    MS_1_16  = 4,
    MS_1_32  = 5 // DRV8825 supports up to 1/32 microstepping
} microstep_mode_e;
#endif

typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t timer_channel;

    GPIO_TypeDef *step_port;
    uint32_t step_pin;
    GPIO_TypeDef *dir_port;
    uint32_t dir_pin;

    // OPTIONAL
    GPIO_TypeDef *en_port;
    uint32_t en_pin;
    GPIO_TypeDef *fault_port;
    uint32_t fault_pin;

    #if STEPPER_MICROSTEP_ENABLED
    GPIO_TypeDef *ms_ports[3];
    uint32_t ms_pins[3];
    #endif

    struct
    {
        stepper_status_e status;
        uint32_t counter;
        uint32_t step;
        #if STEPPER_MICROSTEP_ENABLED
        microstep_mode_e microstep_mode;
        #endif
    } stepper_set;
} stepper_config_t;

typedef struct
{
    // H-Bridge 1 (Coil A)
    GPIO_TypeDef *ph1_port;
    uint16_t ph1_pin;
    GPIO_TypeDef *en1_port;
    uint16_t en1_pin;

    // H-Bridge 2 (Coil B)
    GPIO_TypeDef *ph2_port;
    uint16_t ph2_pin;
    GPIO_TypeDef *en2_port;
    uint16_t en2_pin;

    // Common pins
    GPIO_TypeDef *sleep_port;
    uint16_t sleep_pin;
    GPIO_TypeDef *fault_port;
    uint16_t fault_pin;

    uint8_t current_step; // Tracks position in the 4-step sequence
} stepper_ph_en_t;

void Stepper_Init(stepper_config_t* config);
#if STEPPER_MICROSTEP_ENABLED
void Stepper_SetMicrostep(stepper_config_t *config, microstep_mode_e mode);
#endif
void Stepper_StartMotor(stepper_config_t* config, stepper_direction_e direction, uint32_t steps);
void Stepper_StopMotor(stepper_config_t* config);

void Stepper_PhaseEnableControl(stepper_ph_en_t* control, stepper_direction_e direction);

#endif /* STEPPER_H */