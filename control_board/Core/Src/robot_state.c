/**
 * @file robot_state.c
 * @brief Robot state machine implementation.
 */

#include "robot_state.h"
#include "drv88xx.h"
#include "robot_control.h"
#include "robot_config.h"

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

extern volatile uint16_t adc_dma_buf[ADC_BUFFER_SIZE];

// -- Motor Handles -----------------------------------------------------------

extern stepper_ctrl_t stepper_underpass;

extern motor_ctrl_t dc_pitch;
extern motor_ctrl_t dc_roll;
extern motor_ctrl_t dc_yaw;
extern motor_ctrl_t clamp;

// -- Function Prototypes -----------------------------------------------------

typedef enum {
  STATE_TOP,

  STATE_IDLE,
  STATE_RECEIVING_SPI,
  STATE_RECEIVING_USB,

  STATE_MOVING,
  STATE_HOME,
  STATE_CLAMPING,
  STATE_UNCLAMPING,
  STATE_ARM,
  STATE_SHARPENING,
  STATE_FAULT,
  STATE_ESTOP,
  STATE_INVALID = 0xFF,
} robot_state_id_t;

static tiny_hsm_result_t state_receiving_spi(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_receiving_usb(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_moving(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_mv_home(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_clamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_unclamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_arm(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_sharpening(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);

static tiny_hsm_result_t state_estop(tiny_hsm_t *hsm, tiny_hsm_signal_t sig, const void *data);

// -- State Machine Configuration ---------------------------------------------

static robot_state_id_t get_current_state(void) {
  if (robot_hsm.current == (tiny_hsm_state_t)state_receiving_spi) return STATE_RECEIVING_SPI;
  if (robot_hsm.current == (tiny_hsm_state_t)state_receiving_usb) return STATE_RECEIVING_USB;
  if (robot_hsm.current == (tiny_hsm_state_t)state_moving) return STATE_MOVING;
  if (robot_hsm.current == (tiny_hsm_state_t)state_mv_home) return STATE_HOME;
  if (robot_hsm.current == (tiny_hsm_state_t)state_clamping) return STATE_CLAMPING;
  if (robot_hsm.current == (tiny_hsm_state_t)state_unclamping) return STATE_UNCLAMPING;
  if (robot_hsm.current == (tiny_hsm_state_t)state_arm) return STATE_ARM;
  if (robot_hsm.current == (tiny_hsm_state_t)state_sharpening) return STATE_SHARPENING;

  if (robot_hsm.current == (tiny_hsm_state_t)state_estop) return STATE_ESTOP;
  return STATE_INVALID;
}

static const tiny_hsm_state_descriptor_t robot_hsm_states[] = {
    {.state = (tiny_hsm_state_t)state_receiving_spi,
     .parent = (tiny_hsm_state_t)tiny_hsm_no_parent},
    {.state = (tiny_hsm_state_t)state_receiving_usb,
     .parent = (tiny_hsm_state_t)tiny_hsm_no_parent},

    {.state = (tiny_hsm_state_t)state_moving,
     .parent = (tiny_hsm_state_t)tiny_hsm_no_parent},
    {.state = (tiny_hsm_state_t)state_mv_home,
     .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_clamping,
     .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_unclamping,
     .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_arm,
     .parent = (tiny_hsm_state_t)state_moving},
    {.state = (tiny_hsm_state_t)state_sharpening,
     .parent = (tiny_hsm_state_t)state_moving},

    {.state = (tiny_hsm_state_t)state_estop,
     .parent = (tiny_hsm_state_t)tiny_hsm_no_parent},
};

static const tiny_hsm_configuration_t robot_hsm_config = {
    .states = robot_hsm_states,
    .state_count =
        sizeof(robot_hsm_states) / sizeof(tiny_hsm_state_descriptor_t),
};

// -- IDLE STATE --------------------------------------------------------------

static void handle_spi_data(const uint8_t *data) {
  // data is always one byte representing the knife type
  knife_type_t knife_type = (knife_type_t)data[0];
  Ctrl_SetKnifeType(knife_type);
}

static tiny_hsm_result_t state_receiving_spi(tiny_hsm_t *hsm,
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
    // handle SPI data
    handle_spi_data(data);

    // once we set knife type, wait for USB command that knife is seen
    tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_receiving_usb);
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_state_t handle_usb_data(const uint8_t *data, size_t len) {
  // TODO: figure out USB data format and implement handling logic
  // We want to be receiving continuous updates with reverse kinematics
  // data about the target velocity of motors
  // This might be moved to reading the buffer in the movement state instead of
  // as a receiving state since we will be receiving data continuously while moving

  // if we detect the knife, transition to the clamping state to clamp the knife
  if (0) {
    return (tiny_hsm_state_t)state_clamping;
  }

  // TODO: once we receive kinematics data, transition to moving state and start executing moves
  if (0) {
    // save kinematics data to appropriate PVs for use in movement state
    return (tiny_hsm_state_t)state_moving;
  }

  // TODO: once we finish sharpening, send KNIFE DONE to the HMI
  if (0) {
    tiny_ring_buffer_insert(
      &spi_tx_buf, (uint8_t*)ROBOT_HMI_CMD_KNIFE_DONE);
  }
  return (tiny_hsm_state_t)state_moving; // TODO: transition to appropriate state based on command received
}

static tiny_hsm_result_t state_receiving_usb(tiny_hsm_t *hsm,
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
    tiny_hsm_state_t next_state = handle_usb_data(data, usb_rx_len);

    if (next_state) {
      tiny_hsm_transition(hsm, next_state);
    }
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

  case SIG_MOVE_COMPLETE:
    // TODO
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t
state_mv_home(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  if (signal == tiny_hsm_signal_entry) {
    // TODO: start homing sequence

    return tiny_hsm_result_signal_consumed;
  }
  return tiny_hsm_result_signal_deferred;
}

static tiny_hsm_result_t state_clamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      // send knife clamped command to HMI
      tiny_ring_buffer_insert(
        &spi_tx_buf, (uint8_t*)ROBOT_HMI_CMD_KNIFE_CLAMPED);
      return tiny_hsm_result_signal_consumed;

    case SIG_KNIFE_CLAMPED:
    // TODO: decide the next state
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_receiving_usb);
      return tiny_hsm_result_signal_consumed;
    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_unclamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement unclamping state
  return tiny_hsm_result_signal_deferred;
}

static tiny_hsm_result_t state_arm(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement arm movement state and rename signal
  // maybe have substates for different arm movements?
  return tiny_hsm_result_signal_deferred;
}

static tiny_hsm_result_t state_sharpening(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement sharpening state
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
  tiny_hsm_init((tiny_hsm_t*)&robot_hsm, &robot_hsm_config, (tiny_hsm_state_t)state_receiving_spi);
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
  case STATE_MOVING:
  case STATE_HOME:
  case STATE_CLAMPING:
    // check if the current is within threshold of target and if so,
    // send move complete signal and transition to idle
    if (clamp.current_ma > CURRENT_THRESHOLD_KNIFECLAMP_MA) {
      RobotState_SendSignal(SIG_KNIFE_CLAMPED, NULL);
    }
    break;

  case STATE_UNCLAMPING:
    // check if the limit switch is triggered
    if (clamp.limit_triggered) {
      // TODO: decide what to do after clamping
      tiny_hsm_transition((tiny_hsm_t*)&robot_hsm, (tiny_hsm_state_t)state_receiving_spi);
    }

  case STATE_ARM:
  case STATE_SHARPENING:
    // TODO: add logic to check if move is complete based on encoder readings and target position
    break;

  case STATE_RECEIVING_SPI:
    while (tiny_ring_buffer_count(&spi_rx_buf) > 0) {
      uint8_t byte;
      tiny_ring_buffer_remove(&spi_rx_buf, &byte);
      if (byte == ROBOT_HMI_CMD_NONE) continue; // ignore empty bytes
      RobotState_SendSignal(SIG_RECEIVED_SPI, &byte);
    }
    break;

  case STATE_RECEIVING_USB:
    if (usb_rx_flag) {
      RobotState_SendSignal(SIG_RECEIVED_USB, usb_rx_buf);
      usb_rx_flag = 0; // reset flag after processing
    }
    break;
    
  case STATE_FAULT: break; // TODO: maybe check if fault condition cleared and transition to idle?
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