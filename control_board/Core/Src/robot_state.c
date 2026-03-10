/**
 * @file robot_state.c
 * @brief Robot state machine implementation.
 */

#include "robot_state.h"
#include "drv88xx.h"
#include "robot_control.h"
#include "tiny_hsm.h"
#include "tiny_ring_buffer.h"
#include "robot_control.h"
#include "robot_config.h"

#include <string.h>

// -- Defines -----------------------------------------------------------------
// timeout for moves in milliseconds, used for watchdog timer
#define MOVE_TIMEOUT_MS 10000

// TODO: figure out appropriate buffer size
#define RX_BUF_SIZE 32

// -- Typedefs ----------------------------------------------------------------

// -- PVs ---------------------------------------------------------------------

static volatile uint32_t tick_div;

// TODO: DO I NEED A WATCHDOG??
// used to track time spent in moving state for watchdog timer
static uint32_t watchdog_ms = 0;

uint8_t spi_rx_buf[RX_BUF_SIZE];
uint8_t uart_rx_buf[RX_BUF_SIZE];

tiny_ring_buffer_t spi_buf;
tiny_ring_buffer_t uart_buf;

// -- Function Prototypes -----------------------------------------------------

static tiny_hsm_result_t state_top(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_idle(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_receiving_spi(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_receiving_uart(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_moving(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_mv_home(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_clamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_unclamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_arm(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_sharpening(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_fault(tiny_hsm_t *hsm, tiny_hsm_signal_t sig, const void *data);
static tiny_hsm_result_t state_estop(tiny_hsm_t *hsm, tiny_hsm_signal_t sig, const void *data);

// -- State Machine Configuration ---------------------------------------------

static const tiny_hsm_state_descriptor_t robot_hsm_states[] = {
    {.state = (tiny_hsm_state_t)state_top, .parent = tiny_hsm_no_parent},
    {.state = (tiny_hsm_state_t)state_idle,
     .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_receiving_spi,
     .parent = (tiny_hsm_state_t)state_idle},
    {.state = (tiny_hsm_state_t)state_receiving_uart,
     .parent = (tiny_hsm_state_t)state_idle},

    {.state = (tiny_hsm_state_t)state_moving,
     .parent = (tiny_hsm_state_t)state_top},
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

    {.state = (tiny_hsm_state_t)state_fault,
     .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_estop,
     .parent = (tiny_hsm_state_t)state_top},
};

static const tiny_hsm_configuration_t robot_hsm_config = {
    .states = robot_hsm_states,
    .state_count =
        sizeof(robot_hsm_states) / sizeof(tiny_hsm_state_descriptor_t),
};

tiny_hsm_t robot_hsm;

// -- TOP STATE ---------------------------------------------------------------

static tiny_hsm_result_t state_top(tiny_hsm_t *hsm, tiny_hsm_signal_t signal,
                                   const void *data) {
  switch (signal) {
  case SIG_CMD_ESTOP:
    // TODO: handle emergency stop command

    tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_estop);
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

// -- IDLE STATE --------------------------------------------------------------

static tiny_hsm_result_t state_idle(tiny_hsm_t *hsm, tiny_hsm_signal_t signal,
                                    const void *data) {
  switch (signal) {
  case tiny_hsm_signal_entry:
    // immediately wait for user input on SPI 
    // tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_receiving_spi);
    // TODO: put test code here
    MotorCtrl_Enable(&dc_pitch);
    MotorCtrl_SetTarget(&dc_pitch, 1);

    return tiny_hsm_result_signal_consumed;

  case tiny_hsm_signal_exit:
    return tiny_hsm_result_signal_consumed;

  case SIG_CMD_RECEIVE_SPI:
    // TODO: handle user input
    tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_receiving_spi);
    return tiny_hsm_result_signal_consumed;

  case SIG_CMD_RECEIVE_UART:
    // TODO: handle camera input
    tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_receiving_uart);
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

static void handle_spi_data(const uint8_t *data, size_t len) {
  // TODO: figure out SPI data format and implement handling logic
  // we want to save data based on the knife type
  // such as target bevel angle
}

static tiny_hsm_result_t state_receiving_spi(tiny_hsm_t *hsm,
                                             tiny_hsm_signal_t signal,
                                             const void *data) {
  switch (signal) {
  case tiny_hsm_signal_entry:
    tiny_ring_buffer_clear(&spi_buf);
    return tiny_hsm_result_signal_consumed;

  case tiny_hsm_signal_exit:
    return tiny_hsm_result_signal_consumed;

  case SIG_RECEIVED_SPI:
    // handle SPI data
    handle_spi_data(data, /* len */ 0);
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

static void handle_uart_data(const uint8_t *data, size_t len) {
  // TODO: figure out UART data format and implement handling logic
  // We want to be receiving continuous updates with reverse kinematics
  // data about the target velocity of motors
  // This might be moved to reading the buffer in the movement state instead of
  // as a receiving state since we will be receiving data continuously while moving
  // talk to Hayden abt this
}

static tiny_hsm_result_t state_receiving_uart(tiny_hsm_t *hsm,
                                              tiny_hsm_signal_t signal,
                                              const void *data) {
  switch (signal) {
  case tiny_hsm_signal_entry:
    tiny_ring_buffer_clear(&uart_buf);
    return tiny_hsm_result_signal_consumed;

  case tiny_hsm_signal_exit:
    return tiny_hsm_result_signal_consumed;

  case SIG_RECEIVED_UART:
    // handle UART data
    handle_uart_data(data, /* len */ 0);
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
    return tiny_hsm_result_signal_consumed;

  case tiny_hsm_signal_exit:
    watchdog_ms = 0; // reset watchdog on exit from moving state
    return tiny_hsm_result_signal_consumed;

  case SIG_MOVE_COMPLETE:
    if (/* TODO: check if more moves are queued */ 0) {
      // pop next move from queue and execute
      // tiny_hsm_send_signal(hsm, q, NULL);
    } else {
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_idle);
    }
    return tiny_hsm_result_signal_consumed;

  case SIG_CMD_HOME:
  case SIG_CMD_CLAMP:
  case SIG_CMD_UNCLAMP:
  case SIG_CMD_ARM:
  case SIG_CMD_SHARPEN:
    // queue command to be executed after current move completes
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t
state_mv_home(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  if (signal == tiny_hsm_signal_entry) {
    // TODO: start homing sequence

    watchdog_ms = MOVE_TIMEOUT_MS;
    return tiny_hsm_result_signal_consumed;
  }
  return tiny_hsm_result_signal_deferred;
}

static tiny_hsm_result_t state_clamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  // TODO: implement clamping state
  return tiny_hsm_result_signal_deferred;
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

static tiny_hsm_result_t state_fault(tiny_hsm_t *hsm, tiny_hsm_signal_t sig,
                                     const void *data) {
  switch (sig) {
  case tiny_hsm_signal_entry:
    // TODO: handle fault entry (e.g. stop all motors, disable outputs, etc.)
    return tiny_hsm_result_signal_consumed;
  case tiny_hsm_signal_exit:
    // TODO: handle fault exit (e.g. re-enable outputs, etc.)
    return tiny_hsm_result_signal_consumed;

  case SIG_FAULT_CLEARED:
    // TODO: handle fault cleared (e.g. check if its safe to resume operation,
    // etc.)
    tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_idle);
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

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
  memset(&spi_rx_buf, 0, sizeof(spi_rx_buf));
  memset(&uart_rx_buf, 0, sizeof(uart_rx_buf));
  watchdog_ms = 0;
  tick_div = 0;

  tiny_ring_buffer_init(&spi_buf, spi_rx_buf, RX_BUF_SIZE, sizeof(uint8_t));
  tiny_ring_buffer_init(&uart_buf, uart_rx_buf, RX_BUF_SIZE, sizeof(uint8_t));

  tiny_hsm_init(&robot_hsm, &robot_hsm_config, (tiny_hsm_state_t)state_idle);
}

void RobotState_Tick(void) {
  // TODO: call this from a timer interrupt every 10 ms
}

// -- HAL Callbacks -----------------------------------------------------------

void HAL_SYSTICK_Callback(void) {
  tick_div++;
  if (tick_div < 10)
    return;

  tick_div = 0;

  // RobotState_Tick();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *h) {
  if (h->Instance == SPI1) {
    tiny_ring_buffer_insert(&spi_buf, spi_rx_buf);
    HAL_SPI_Receive_IT(h, spi_rx_buf, 1);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
  if (h->Instance == USART1) {
    tiny_ring_buffer_insert(&uart_buf, uart_rx_buf);
    HAL_UART_Receive_IT(h, uart_rx_buf, 1);
  }
}
