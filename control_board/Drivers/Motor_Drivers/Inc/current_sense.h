#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "stm32g4xx_hal.h"

#include <stdbool.h>

#define CURRENT_SENSE_ADC_RESOLUTION 4096.0f // 12-bit ADC
#define CURRENT_SENSE_VREF_MV 3300.0f // Reference voltage in mill

typedef struct {
    ADC_HandleTypeDef *adc_instance; // ADC instance (e.g., ADC1)
    uint32_t adc_channel;      // ADC channel for current sensing
    uint8_t shunt_resistor_mohm; // Shunt resistor value in milliohms for current calculation
} current_sense_config_t;

/**
 * @brief Get the current in milliamperes
 * @param config Current sense configuration
 * @return Current in milliamperes
 */
float CurrentSense_GetCurrentmA(current_sense_config_t *config);

/** 
 * @brief Check if the current exceeds the threshold
 * @param config Current sense configuration
 * @param threshold_ma Overcurrent threshold in milliamperes
 * @return true if overcurrent is detected, false otherwise
 */
bool CurrentSense_CheckOvercurrent(current_sense_config_t *config, float threshold_ma);

#endif /* CURRENT_SENSE_H */