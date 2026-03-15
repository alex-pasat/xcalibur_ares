#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "stm32g4xx_hal.h"

#include <stdbool.h>

typedef struct {
    ADC_HandleTypeDef *adc_instance; // ADC instance (e.g., ADC1)
    unsigned long adc_index;      // ADC channel for current sensing
    uint8_t shunt_resistor_mohm; // Shunt resistor value in milliohms for current calculation
} current_sense_config_t;

/**
 * @brief Get the current in milliamperes
 * @param config Current sense configuration
 * @return Current in milliamperes
 */
uint32_t CurrentSense_GetCurrentmA(current_sense_config_t *config);

#endif /* CURRENT_SENSE_H */