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
  SIG_ESTOP = tiny_hsm_signal_user_start + 1,
  SIG_FAULT,
  SIG_FAULT_CLEARED,

  SIG_CMD_RECEIVE_SPI,
  SIG_CMD_RECEIVE_USB,

  SIG_RECEIVED_SPI,
  SIG_RECEIVED_USB,

  SIG_AWAIT_KNIFE_DETECTION,
  SIG_KNIFE_DETECTED,
  SIG_KNIFE_REMOVED,

  SIG_KNIFE_CLAMPED,

  SIG_MOVE_COMPLETE,

} robot_signal_t;

typedef enum {
  ROBOT_HMI_CMD_NONE           = 0x00,
  ROBOT_HMI_CMD_KNIFE_CLAMPED  = 0x01,
  ROBOT_HMI_CMD_KNIFE_DONE     = 0x02,
  ROBOT_HMI_CMD_KNIFE_REMOVED  = 0x03,
  // TODO: add more commands as needed
} robot_hmi_command_t;

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
 * @brief Periodic 10ms tick — polls sensors, watchdog, and move completion.
 *        Called from main loop when TIM6 10ms flag is set.
 *        Do not call directly from interrupt context.
 */
void RobotState_Tick(void);

#endif /* ROBOT_STATE_H */