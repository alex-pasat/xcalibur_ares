#ifndef ENCODER_H
#define ENCODER_H

// -- INCLUDES ----------------------------------------------------------------
#include "stm32g4xx.h"

#include <stdint.h>

// -- TYPE DEFINITIONS --------------------------------------------------------
typedef enum {
    ENC_STATE_00 = 0b00,    // A=0, B=0
    ENC_STATE_01 = 0b01,    // A=0, B=1
    ENC_STATE_10 = 0b10,    // A=1, B=0
    ENC_STATE_11 = 0b11,    // A=1, B=1
} enc_ab_state_t;

typedef struct {
    // -- Encoder GPIO pins --
    GPIO_TypeDef *enc_a_port;
    uint32_t enc_a_pin;
    GPIO_TypeDef *enc_b_port;
    uint32_t enc_b_pin;

    // -- State --
    enc_ab_state_t last_state;  // Last AB state, used to determine direction
    int32_t count;              // Absolute position in ticks
    uint32_t counts_per_rev;    // Encoder resolution (ticks per revolution)

    float velocity_rps;         // Current velocity in revolutions per second
} enc_config_t;

#endif /* ENCODER_H */