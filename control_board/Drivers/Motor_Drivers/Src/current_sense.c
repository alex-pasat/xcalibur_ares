/**
 * @file    current_sense.c
 * @brief   Current sense implementation.
 */
#include "current_sense.h"

// ADC DMA buffer defined in robot_config.c
extern volatile uint16_t adc_dma_buf[7]; 

static uint32_t adc_to_current_ma(uint16_t adc_value, uint8_t shunt_resistor_mohm) {
    // Convert ADC value to voltage
    float voltage_mv = ((float)adc_value / CURRENT_SENSE_ADC_RESOLUTION) * CURRENT_SENSE_VREF_MV;
    // Calculate current using Ohm's law: I = V / R
    float current_ma = voltage_mv / shunt_resistor_mohm; 
    return (uint32_t)(current_ma * 1000.0f); // Convert to milliamps and scale
}

uint32_t CurrentSense_GetCurrentmA(current_sense_config_t *config) {
    // Get the latest ADC value from the DMA buffer
    uint16_t adc_value = adc_dma_buf[config->adc_channel];
    return adc_to_current_ma(adc_value, config->shunt_resistor_mohm);
}
