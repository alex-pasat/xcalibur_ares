/**
 * @file encoder.c
 * @brief Encoder reading implementation
 */

 // -- INCLUDES ---------------------------------------------------------------
#include "encoder.h"

// -- PRIVATE VARIABLES -------------------------------------------------------

static const int8_t QEM[4][4] = {
        {  0, -1, +1,  0 },   // last: 00
        { +1,  0,  0, -1 },   // last: 01
        { -1,  0,  0, +1 },   // last: 10
        {  0, +1, -1,  0 },   // last: 11
};

// -- FUNCTION DEFINITIONS ----------------------------------------------------

void Encoder_Init(enc_config_t *enc) {
    uint8_t a_state = HAL_GPIO_ReadPin(enc->enc_a_port, enc->enc_a_pin) ? 1 : 0;
    uint8_t b_state = HAL_GPIO_ReadPin(enc->enc_b_port, enc->enc_b_pin) ? 1 : 0;
    enc->last_state = (a_state << 1) | b_state;
    enc->count = 0;
    enc->velocity_rps = 0.0f;
}

void Encoder_ComputeVelocity(enc_config_t *enc, int32_t delta_ticks, float dt_s) {
    enc->velocity_rps = ((float)delta_ticks / enc->counts_per_rev) / dt_s;
}

float DRV8251_Encoder_GetAngleDeg(const enc_config_t *enc) {
    int32_t ticks_mod = enc->count % (int32_t)enc->counts_per_rev;
    if (ticks_mod < 0) ticks_mod += enc->counts_per_rev;
    return ((float)ticks_mod / (float)enc->counts_per_rev) * 360.0f;
}

void DRV8251_Encoder_Reset(enc_config_t *enc) {
    enc->count      = 0;
    enc->last_state = 0;
}

void Encoder_EXTI_Callback(enc_config_t *enc) {
    uint8_t a_state = HAL_GPIO_ReadPin(enc->enc_a_port, enc->enc_a_pin) ? 1 : 0;
    uint8_t b_state = HAL_GPIO_ReadPin(enc->enc_b_port, enc->enc_b_pin) ? 1 : 0;
    enc_ab_state_t new_state = (a_state << 1) | b_state;
    enc->count += QEM[enc->last_state][new_state];
    enc->last_state = new_state;
}