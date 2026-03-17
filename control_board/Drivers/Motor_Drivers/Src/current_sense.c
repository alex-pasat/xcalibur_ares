/**
 * @file    current_sense.c
 * @brief   Current sense implementation.
 */
#include "current_sense.h"

#define CURRENT_SENSE_ADC_RESOLUTION 4096.0f // 12-bit ADC
#define CURRENT_SENSE_VREF_MV 3300.0f // Reference voltage in mill
#define CURRENT_SENSE_GAIN 50.0f

// ADC DMA buffer defined in robot_config.c
extern volatile uint16_t adc_dma_buf[7]; 

static uint32_t adc_to_current_ma(uint16_t adc_value, uint8_t shunt_resistor_mohm) {
    float v_out_mv = ((float)adc_value / CURRENT_SENSE_ADC_RESOLUTION) * CURRENT_SENSE_VREF_MV;
    float v_shunt_mv = v_out_mv / CURRENT_SENSE_GAIN; 
    float current_ma = (v_shunt_mv / (float)shunt_resistor_mohm) * 1000.0f;
    
    return (uint32_t)current_ma;
}

uint32_t CurrentSense_GetCurrentmA(current_sense_config_t *config) {
    // Get the latest ADC value from the DMA buffer
    uint16_t adc_value = adc_dma_buf[config->adc_index];
    return adc_to_current_ma(adc_value, config->shunt_resistor_mohm);
}
