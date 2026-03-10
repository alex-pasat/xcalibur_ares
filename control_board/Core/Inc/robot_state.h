/**
 * @file    robot_state.h
 * @brief   Robot arm state machine
 */

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

// -- Includes ----------------------------------------------------------------
#include "tiny_hsm.h"

// -- Type Definitions --------------------------------------------------------

typedef enum {
  SIG_CMD_ESTOP = tiny_hsm_signal_user_start + 1,
  SIG_FAULT,
  SIG_FAULT_CLEARED,

  SIG_CMD_RECEIVE_SPI,
  SIG_CMD_RECEIVE_UART,
  SIG_RECEIVED_SPI,
  SIG_RECEIVED_UART,

  SIG_MOVE_COMPLETE,
  SIG_CMD_HOME,
  SIG_CMD_CLAMP,
  SIG_CMD_UNCLAMP,
  SIG_CMD_ARM, // TODO: rename to something more descriptive of the actual arm
               // movement
  SIG_CMD_SHARPEN,

  SIG_LIMIT_TRIGGERED,
  SIG_HALL_TRIGGERED,
} robot_signal_t;

//-- Function Prototypes ------------------------------------------------------

/**
 * @brief Initializes the robot HSM.
 * Call once before the main loop, after peripheral init.
 */
void RobotState_Init(void);

/**
 * @brief Sends a signal to the robot HSM.
 * @param sig The signal to send.
 * @param data Pointer to the data associated with the signal.
 */
void RobotState_SendSignal(robot_signal_t sig, const void *data);

/**
 * @brief Periodic 10 ms tick — polls joint completion and watchdog.
 *        Call from SysTick or a timer interrupt.
 */
void RobotState_Tick(void);

#endif /* ROBOT_STATE_H */