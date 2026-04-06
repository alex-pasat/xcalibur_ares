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

  SIG_TICK,
  SIG_MOVE_COMPLETE,
} robot_signal_t;

typedef enum {
  ROBOT_HMI_CMD_NONE               = 0x00,
  ROBOT_HMI_CMD_KNIFE_CLAMPED      = 0x01,
  ROBOT_HMI_CMD_KNIFE_DONE         = 0x02,
  ROBOT_HMI_CMD_KNIFE_REMOVED      = 0x03,
  ROBOT_HMI_CMD_KNIFE_NOT_DETECTED = 0x04,
  ROBOT_HMI_CMD_RPI_DETECTED       = 0x05,
} robot_hmi_command_t;

typedef enum {
  NONE = 0x00,
  KNIFETYPE_CHEF,
  KNIFETYPE_PARING,
  KNIFETYPE_GYOTO,
  KNIFETYPE_JAP_UTIL,
  N_KNIFE_TYPES
} knife_type_t;

typedef enum {
  KNIFETYPE_NONE,
  KNIFETYPE_CHEF_CMD,
  KNIFETYPE_PARING_CMD,
  KNIFETYPE_GYOTO_CMD,
  KNIFETYPE_JAP_UTIL_CMD,
  REQUESTING_DATA = 0x20,
} hmi_to_robot_command_t;

typedef struct {
  float target_bevel_angle_deg;
  // etc..
} sharpening_parameters_t;

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

void RobotState_DecoderHighFreq(void);

/**
 *  @brief Decodes incoming signals and updates the robot state.
 */
void RobotState_Decoder(void);

#endif /* ROBOT_STATE_H */