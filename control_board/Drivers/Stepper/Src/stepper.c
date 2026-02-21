#include "stepper.h"

extern stepper_config_t* g_stepper_config;

void Stepper_Init(stepper_config_t* config) {
    HAL_GPIO_WritePin(config->en_port, config->en_pin, GPIO_PIN_SET); // Disable stepper by default

    #if STEPPER_MICROSTEP_ENABLED
    Stepper_SetMicrostep(0, MS_FULL); // Default to full step
    #endif

    config->stepper_set.status = STEPPER_OFF;
    config->stepper_set.counter = 0;
    config->stepper_set.step = 0;
}

#if STEPPER_MICROSTEP_ENABLED
void Stepper_SetMicrostep(stepper_config_t *config, microstep_mode_e mode) {
    uint8_t m_bits = 0;

    // Mapping for DRV8825 (M0, M1, M2)
    switch(mode) {
        case MS_FULL: m_bits = 0b000; break;
        case MS_HALF: m_bits = 0b100; break;
        case MS_1_4:  m_bits = 0b010; break;
        case MS_1_8:  m_bits = 0b110; break;
        case MS_1_16: m_bits = 0b001; break;
        case MS_1_32: m_bits = 0b111; break;
        default:      m_bits = 0b000;
    }

    HAL_GPIO_WritePin(config->ms_ports[0], config->ms_pins[0], (m_bits & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(config->ms_ports[1], config->ms_pins[1], (m_bits & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(config->ms_ports[2], config->ms_pins[2], (m_bits & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
#endif

void Stepper_StartMotor(stepper_config_t* config, stepper_direction_e direction, uint32_t steps) {
    if(steps == 0) return;

    if(HAL_GPIO_ReadPin(config->fault_port, config->fault_pin) == GPIO_PIN_RESET) {
        // Fault condition, do not start motor
        return;
    }

    HAL_GPIO_WritePin(config->dir_port, config->dir_pin, (direction == CW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(config->en_port, config->en_pin, GPIO_PIN_RESET);

    config->stepper_set.status = STEPPER_ON;
    config->stepper_set.counter = 0;
    config->stepper_set.step = steps;
}

void Stepper_StopMotor(stepper_config_t* config) {
    HAL_TIM_PWM_Stop(config->timer, config->timer_channel);
    HAL_GPIO_WritePin(config->en_port, config->en_pin, GPIO_PIN_SET);
    config->stepper_set.status = STEPPER_OFF;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if(g_stepper_config->stepper_set.status == STEPPER_ON) {
        g_stepper_config->stepper_set.counter++;
        if(g_stepper_config->stepper_set.counter >= g_stepper_config->stepper_set.step) {
            Stepper_StopMotor(g_stepper_config);
        }
    }
}

void Stepper_PH_EN_Step(stepper_ph_en_t *config, stepper_direction_e dir) {
    // 1. Update the step index
    if (dir == CW) {
        config->current_step = (config->current_step + 1) % 4;
    } else {
        config->current_step = (config->current_step == 0) ? 3 : config->current_step - 1;
    }

    // 2. Apply Phase/Enable Logic for Full Stepping
    // Note: We keep Enable HIGH for current to flow
    switch (config->current_step) {
        case 0: // A+ B+
            HAL_GPIO_WritePin(config->ph1_port, config->ph1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(config->ph2_port, config->ph2_pin, GPIO_PIN_SET);
            break;
        case 1: // A- B+
            HAL_GPIO_WritePin(config->ph1_port, config->ph1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->ph2_port, config->ph2_pin, GPIO_PIN_SET);
            break;
        case 2: // A- B-
            HAL_GPIO_WritePin(config->ph1_port, config->ph1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(config->ph2_port, config->ph2_pin, GPIO_PIN_RESET);
            break;
        case 3: // A+ B-
            HAL_GPIO_WritePin(config->ph1_port, config->ph1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(config->ph2_port, config->ph2_pin, GPIO_PIN_RESET);
            break;
    }

    // Ensure bridges are enabled
    HAL_GPIO_WritePin(config->en1_port, config->en1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(config->en2_port, config->en2_pin, GPIO_PIN_SET);\
    
    __ASM volatile ("nop"); // Short delay for signals to stabilize
}