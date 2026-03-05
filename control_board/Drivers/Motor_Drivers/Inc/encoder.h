#ifndef ENCODER_H
#define ENCODER_H

// -- INCLUDES ----------------------------------------------------------------
#include "stm32g4xx.h"

#include <stdint.h>

// -- TYPE DEFINITIONS --------------------------------------------------------
typedef enum {
  ENC_STATE_00 = 0b00, // A=0, B=0
  ENC_STATE_01 = 0b01, // A=0, B=1
  ENC_STATE_10 = 0b10, // A=1, B=0
  ENC_STATE_11 = 0b11, // A=1, B=1
} enc_ab_state_t;

typedef struct {
  // -- Encoder GPIO pins --
  GPIO_TypeDef *enc_a_port;
  uint32_t enc_a_pin;
  GPIO_TypeDef *enc_b_port;
  uint32_t enc_b_pin;

  // -- State --
  enc_ab_state_t last_state; // Last AB state, used to determine direction
  int32_t count;             // Absolute position in ticks
  uint32_t counts_per_rev;   // Encoder resolution (ticks per revolution)

  float velocity_rps; // Current velocity in revolutions per second
} enc_config_t;

// -- FUNCTION PROTOTYPES -----------------------------------------------------
void Encoder_Init(enc_config_t *enc);

float Encoder_ComputeVelocity(enc_config_t *enc, int32_t delta_ticks,
                              float dt_s);

float Encoder_GetAngleDeg(const enc_config_t *enc);

void Encoder_Reset(enc_config_t *enc);


/**
  * @brief EXTI callback to be called from HAL_GPIO_EXTI_Callback() in main.c
  *        This will update the encoder count based on the state of the A and B
  *        pins. It uses a quadrature encoder matrix to determine the direction of
  *        rotation and update the count accordingly.
  * @param enc Pointer to the encoder configuration struct for the encoder being updated
  * @note This function should be called from the EXTI interrupt handler for the
  *       encoder pins. You will need to set up the EXTI interrupts for the
  *       encoder pins in your main.c and call this function with the appropriate
  *       encoder config struct when an interrupt occurs on those pins.
  * @note This function assumes that both A and B pins are configured to trigger
  *       interrupts on both rising and falling edges, so it will be called on
  *       every change of state of either pin.
  */
void Encoder_EXTI_Callback(enc_config_t *enc);

#endif /* ENCODER_H */