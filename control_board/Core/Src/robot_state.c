/**
 * @file robot_state.c
 * @brief Robot state machine implementation.
 */

#include "robot_state.h"
#include "drv88xx.h"
#include "robot_control.h"
#include "robot_config.h"

#include "stm32g491xx.h"
#include "stm32g4xx_hal_spi.h"
#include "tiny_hsm.h"
#include "tiny_ring_buffer.h"
#include "usb_device.h"

#include <stdint.h>
#include <string.h>

// -- Defines -----------------------------------------------------------------
// timeout for moves in milliseconds, used for watchdog timer
#define MOVE_TIMEOUT_MS 10000

// -- PVs ---------------------------------------------------------------------

volatile tiny_hsm_t robot_hsm;

extern SPI_HandleTypeDef hspi1;

static uint8_t spi_raw_rx;
static uint8_t spi_raw_tx;

extern tiny_ring_buffer_t spi_rx_buf;
extern tiny_ring_buffer_t spi_tx_buf;

extern uint8_t usb_rx_buf[64];
extern volatile uint8_t usb_rx_flag;
extern volatile uint32_t usb_rx_len;

extern volatile uint16_t adc_dma_buf[];

// -- Motor Handles -----------------------------------------------------------

extern stepper_ctrl_t stepper_underpass;

extern motor_ctrl_t dc_pitch;
extern motor_ctrl_t dc_roll;
extern motor_ctrl_t dc_yaw;
extern motor_ctrl_t clamp;

// -- Function Prototypes -----------------------------------------------------

typedef enum {
  STATE_TOP,

  STATE_AWAITING_KNIFE_SELECTION,
  STATE_AWAITING_KNIFE_INPUT,
  STATE_AWAITING_GEOMETRY,

  STATE_MOVING,
  STATE_HOME,

  STATE_CLAMPING,
  STATE_UNCLAMPING,

  STATE_MOVE_TO_SIDE_A,
  STATE_MOVE_TO_SIDE_B,
  STATE_MOVE_TO_HOME,

  STATE_SHARPENING_A,
  STATE_SHARPENING_B,

  STATE_ESTOP,
  STATE_INVALID = 0xFF,
} robot_state_id_t;


static tiny_hsm_result_t state_top(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);

static tiny_hsm_result_t state_awaiting_knife_selection(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_awaiting_knife_input(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_awaiting_geometry(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);

static tiny_hsm_result_t state_moving(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_homing(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);

static tiny_hsm_result_t state_clamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_unclamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);

static tiny_hsm_result_t state_mv_to_side_a(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_mv_to_side_b(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_mv_to_home(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);

static tiny_hsm_result_t state_sharpening_a(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_sharpening_b(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);

static tiny_hsm_result_t state_estop(tiny_hsm_t *hsm, tiny_hsm_signal_t sig, const void *data);

// -- State Machine Configuration ---------------------------------------------

static robot_state_id_t get_current_state(void) {
  if (robot_hsm.current == (tiny_hsm_state_t)state_top) return STATE_TOP;

  if (robot_hsm.current == (tiny_hsm_state_t)state_awaiting_knife_selection) return STATE_AWAITING_KNIFE_SELECTION;
  if (robot_hsm.current == (tiny_hsm_state_t)state_awaiting_knife_input) return STATE_AWAITING_KNIFE_INPUT;
  if (robot_hsm.current == (tiny_hsm_state_t)state_awaiting_geometry) return STATE_AWAITING_GEOMETRY;
  if (robot_hsm.current == (tiny_hsm_state_t)state_moving) return STATE_MOVING;
  if (robot_hsm.current == (tiny_hsm_state_t)state_homing) return STATE_HOME;
  if (robot_hsm.current == (tiny_hsm_state_t)state_clamping) return STATE_CLAMPING;
  if (robot_hsm.current == (tiny_hsm_state_t)state_unclamping) return STATE_UNCLAMPING;
  if (robot_hsm.current == (tiny_hsm_state_t)state_mv_to_side_a) return STATE_MOVE_TO_SIDE_A;
  if (robot_hsm.current == (tiny_hsm_state_t)state_mv_to_side_b) return STATE_MOVE_TO_SIDE_B;
  if (robot_hsm.current == (tiny_hsm_state_t)state_mv_to_home) return STATE_MOVE_TO_HOME;
  if (robot_hsm.current == (tiny_hsm_state_t)state_sharpening_a) return STATE_SHARPENING_A;
  if (robot_hsm.current == (tiny_hsm_state_t)state_sharpening_b) return STATE_SHARPENING_B;
  if (robot_hsm.current == (tiny_hsm_state_t)state_estop) return STATE_ESTOP;

  return STATE_INVALID;
}

static const tiny_hsm_state_descriptor_t robot_hsm_states[] = {
    {.state = (tiny_hsm_state_t)state_top, .parent = (tiny_hsm_state_t)tiny_hsm_no_parent},
    
    {.state = (tiny_hsm_state_t)state_awaiting_knife_selection, .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_awaiting_knife_input, .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_awaiting_geometry, .parent = (tiny_hsm_state_t)state_top},

    {.state = (tiny_hsm_state_t)state_moving, .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_homing, .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_clamping, .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_unclamping, .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_mv_to_side_a, .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_mv_to_side_b, .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_mv_to_home, .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_sharpening_a, .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_sharpening_b, .parent = (tiny_hsm_state_t)state_moving},

    {.state = (tiny_hsm_state_t)state_estop, .parent = (tiny_hsm_state_t)state_top},
};

static const tiny_hsm_configuration_t robot_hsm_config = {
    .states = robot_hsm_states,
    .state_count =
        sizeof(robot_hsm_states) / sizeof(tiny_hsm_state_descriptor_t),
};

// -- IDLE STATE --------------------------------------------------------------

static tiny_hsm_result_t state_top(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_awaiting_knife_selection(tiny_hsm_t *hsm,
                                             tiny_hsm_signal_t signal,
                                             const void *data) {
  switch (signal) {
  case tiny_hsm_signal_entry:
    // clear SPI buffer to prepare for new command
    tiny_ring_buffer_clear(&spi_rx_buf);
    return tiny_hsm_result_signal_consumed;

  case tiny_hsm_signal_exit:
    // clear SPI buffer
    tiny_ring_buffer_clear(&spi_rx_buf);
    return tiny_hsm_result_signal_consumed;

  case SIG_RECEIVED_SPI:
    if (data == NULL) {
      return tiny_hsm_result_signal_consumed;
    }

    knife_type_t knife_type = (knife_type_t)*(const uint8_t*)data;
    Ctrl_SetKnifeType(knife_type);

    // once we set knife type, wait for USB command that knife is seen
    tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_awaiting_knife_input);
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_awaiting_knife_input(tiny_hsm_t *hsm,
                                                     tiny_hsm_signal_t signal,
                                                     const void *data) {
  switch (signal) {
  case tiny_hsm_signal_entry:
    // clear USB buffer to prepare for new command
    memset(usb_rx_buf, 0, sizeof(usb_rx_buf));
    usb_rx_flag = 0;
    usb_rx_len = 0;
    return tiny_hsm_result_signal_consumed;

  case tiny_hsm_signal_exit:
    return tiny_hsm_result_signal_consumed;

  case SIG_RECEIVED_USB:
    if (data == NULL || usb_rx_len == 0) {
      return tiny_hsm_result_signal_consumed;
    }

    // TODO: parse USB data and decide next state based on command type


    // TODO: decide on USB data format
    // if knife detected
    if (0) {
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_clamping);
    }

    // if kinematics data received
    if (0) {
      // save kinematics data to appropriate PVs for use in movement state
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_moving);
    }

    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_awaiting_geometry(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

// -- OPERATIONAL STATE -------------------------------------------------------

static tiny_hsm_result_t state_moving(tiny_hsm_t *hsm, tiny_hsm_signal_t signal,
                                      const void *data) {
  switch (signal) {
  case tiny_hsm_signal_entry:
  case tiny_hsm_signal_exit:
    return tiny_hsm_result_signal_consumed;

  case SIG_ESTOP:
    tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_estop);
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_homing(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_clamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      // send KNIFE_CLAMPED command to HMI over SPI
      tiny_ring_buffer_insert(
        &spi_tx_buf, (uint8_t*)ROBOT_HMI_CMD_KNIFE_CLAMPED);

      // send signal to USB task to notify knife is clamped and kinematics data can be sent
      // send here or is USB task?
      USB_SendString((const char*)ROBOT_RPI_REQUEST_KINEMATICS);

      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_awaiting_geometry);
      return tiny_hsm_result_signal_consumed;
    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_unclamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement unclamping state
  return tiny_hsm_result_signal_deferred;
}

static tiny_hsm_result_t state_mv_to_side_a(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement move to side A state
  return tiny_hsm_result_signal_deferred;
}

static tiny_hsm_result_t state_mv_to_side_b(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement move to side B state
  return tiny_hsm_result_signal_deferred;
}

static tiny_hsm_result_t state_mv_to_home(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement move to home state
  return tiny_hsm_result_signal_deferred;
}

static tiny_hsm_result_t state_sharpening_a(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement sharpening on side A state
  return tiny_hsm_result_signal_deferred;
}

static tiny_hsm_result_t state_sharpening_b(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement sharpening on side B state
  return tiny_hsm_result_signal_deferred;
}

// -- FAULT / ESTOP STATE -----------------------------------------------------

static tiny_hsm_result_t state_estop(tiny_hsm_t *hsm, tiny_hsm_signal_t sig,
                                     const void *data) {
  switch (sig) {
  case tiny_hsm_signal_entry:
    // TODO: handle estop entry (e.g. stop all motors immediately, disable
    // outputs, etc.)
    return tiny_hsm_result_signal_consumed;
  case tiny_hsm_signal_exit:
    // TODO: handle estop exit (e.g. re-enable outputs, etc.)
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

// -- State Machine Configuration ---------------------------------------------

void RobotState_Init(void) {
  HAL_SPI_TransmitReceive_IT(&hspi1, &spi_raw_tx, &spi_raw_rx, 1);
  // TODO: either start here or start by homing
  tiny_hsm_init((tiny_hsm_t*)&robot_hsm, &robot_hsm_config, (tiny_hsm_state_t)state_awaiting_knife_selection);
}

void RobotState_SendSignal(robot_signal_t sig, const void *data) {
  tiny_hsm_send_signal((tiny_hsm_t*)&robot_hsm, sig, data);
}


void RobotState_Tick(void) {
  /* TODO: RobotState_Tick()
  - check angle for each joint and if within threshold of target, or if 
    past the hardware limit or limit switch
    send move complete or limit triggered signal and transition to idle or fault state
  - read current sensors and if overcurrent, trigger fault or stop depending on state
    (sometimes this is intended behaviour)
  - if move takes too long, trigger fault? 
  */

  switch (get_current_state()) {
  case STATE_TOP:
    break;

  case STATE_AWAITING_KNIFE_SELECTION:
    while (tiny_ring_buffer_count(&spi_rx_buf) > 0) {
      uint8_t byte;
      tiny_ring_buffer_remove(&spi_rx_buf, &byte);
      if (byte == ROBOT_HMI_CMD_NONE) continue; // ignore empty bytes
      RobotState_SendSignal(SIG_RECEIVED_SPI, &byte);
    }
    break;

  case STATE_AWAITING_KNIFE_INPUT:
  case STATE_AWAITING_GEOMETRY:
    if (usb_rx_flag) {
      RobotState_SendSignal(SIG_RECEIVED_USB, usb_rx_buf);
      usb_rx_flag = 0; // reset flag after processing
    }
    break;

  case STATE_HOME:
    // bring underpass to limit switch
    StepperCtrl_SetTarget(&stepper_underpass, -100000);

    // bring clamp to limit switch

    // bring pitch, roll, yaw to home position hall effect

  case STATE_CLAMPING:
    // check if the current is within threshold of target and if so,
    // send move complete signal and transition to idle
    if (clamp.current_ma > CURRENT_THRESHOLD_KNIFECLAMP_MA) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    }
    break;

  case STATE_UNCLAMPING:
    // check if the limit switch is triggered
    if (clamp.limit_triggered) {
      // TODO: decide what to do after clamping

      tiny_ring_buffer_insert(&spi_tx_buf, (uint8_t*)ROBOT_HMI_CMD_KNIFE_DONE);
      tiny_hsm_transition((tiny_hsm_t*)&robot_hsm, (tiny_hsm_state_t)state_awaiting_knife_selection);
    }
    break;

  case STATE_MOVE_TO_SIDE_A:
  case STATE_MOVE_TO_SIDE_B:
  case STATE_MOVE_TO_HOME:
  case STATE_SHARPENING_A:
  case STATE_SHARPENING_B:
    // check if move is complete based on current and angle thresholds, and if so,
    // send move complete signal and transition to idle
    break;


  case STATE_ESTOP: break; // TODO: maybe check if estop condition cleared and transition to idle or fault?

  default:
    break;
  }
}

// -- HAL Callbacks -----------------------------------------------------------

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    // buffer the received byte into the SPI RX ring buffer
    tiny_ring_buffer_insert(&spi_rx_buf, &spi_raw_rx);

    // pop next byte to transmit from the SPI TX ring buffer, or send empty byte if none
    spi_raw_tx = ROBOT_HMI_CMD_NONE;
    tiny_ring_buffer_remove(&spi_tx_buf, &spi_raw_tx);

    // start next SPI transmit/receive
    HAL_SPI_TransmitReceive_IT(hspi, &spi_raw_tx, &spi_raw_rx, 1);
  }
}

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
  if (h->Instance == USART1) {
    tiny_ring_buffer_insert(&usb_buf, usb_rx_buf);
    HAL_UART_Receive_IT(h, usb_rx_buf, 1);
  }
}
#endif