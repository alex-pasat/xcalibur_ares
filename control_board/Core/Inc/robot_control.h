/**
 * @file    robot_control.h
 * @brief   Robot arm state machine
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

// -- Includes ----------------------------------------------------------------
#include "tiny_hsm.h"

// -- Type Definitions --------------------------------------------------------

typedef enum
{
    ROBOT_STATE_TOP,

    ROBOT_STATE_IDLE,
    ROBOT_STATE_USER_INPUT,
    ROBOT_STATE_CAMERA,

    ROBOT_STATE_MOVING,
    ROBOT_STATE_HOME,
    ROBOT_STATE_CLAMPING,
    ROBOT_STATE_UNCLAMPING,
    ROBOT_STATE_ARM, // TODO: rename to something more descriptive of the actual arm movement
    ROBOT_STATE_SHARPENING,

    ROBOT_STATE_FAULT,
    ROBOT_STATE_ESTOP,
} robot_state_t;

typedef enum
{
    SIG_CMD_ESTOP = tiny_hsm_signal_user_start + 1,
    SIG_FAULT,
    SIG_FAULT_CLEARED,  

    SIG_USER_INPUT,
    SIG_CAMERA_INPUT,

    SIG_MOVE_COMPLETE,
    SIG_CMD_HOME,
    SIG_CMD_CLAMP,
    SIG_CMD_UNCLAMP,
    SIG_CMD_ARM, // TODO: rename to something more descriptive of the actual arm movement
    SIG_CMD_SHARPEN,
} robot_signal_t;

// -- State Machine Configuration ---------------------------------------------

static const tiny_hsm_state_descriptor_t robot_hsm_states[] = {
    {.state = (tiny_hsm_state_t)ROBOT_STATE_TOP, .parent = tiny_hsm_no_parent},
    {.state = (tiny_hsm_state_t)ROBOT_STATE_IDLE, .parent = (tiny_hsm_state_t)ROBOT_STATE_TOP},
    {.state = (tiny_hsm_state_t)ROBOT_STATE_USER_INPUT, .parent = (tiny_hsm_state_t)ROBOT_STATE_IDLE},
    {.state = (tiny_hsm_state_t)ROBOT_STATE_CAMERA, .parent = (tiny_hsm_state_t)ROBOT_STATE_IDLE},

    {.state = (tiny_hsm_state_t)ROBOT_STATE_MOVING, .parent = (tiny_hsm_state_t)ROBOT_STATE_TOP},
    {.state = (tiny_hsm_state_t)ROBOT_STATE_HOME, .parent = (tiny_hsm_state_t)ROBOT_STATE_MOVING},
    {.state = (tiny_hsm_state_t)ROBOT_STATE_CLAMPING, .parent = (tiny_hsm_state_t)ROBOT_STATE_MOVING},
    {.state = (tiny_hsm_state_t)ROBOT_STATE_UNCLAMPING, .parent = (tiny_hsm_state_t)ROBOT_STATE_MOVING},
    {.state = (tiny_hsm_state_t)ROBOT_STATE_ARM, .parent = (tiny_hsm_state_t)ROBOT_STATE_MOVING},
    {.state = (tiny_hsm_state_t)ROBOT_STATE_SHARPENING, .parent = (tiny_hsm_state_t)ROBOT_STATE_MOVING},

    {.state = (tiny_hsm_state_t)ROBOT_STATE_FAULT, .parent = (tiny_hsm_state_t)ROBOT_STATE_TOP},
    {.state = (tiny_hsm_state_t)ROBOT_STATE_ESTOP, .parent = (tiny_hsm_state_t)ROBOT_STATE_TOP},};

//-- Function Prototypes ------------------------------------------------------

/**
 * @brief Initializes the robot HSM.
 * Call once before the main loop, after peripheral init.
 */
void RobotControl_Init(void);

/**
 * @brief Periodic 10 ms tick — polls joint completion and watchdog.
 *        Call from SysTick or a timer interrupt.
 */
void RobotControl_Tick(void);

#endif /* ROBOT_CONTROL_H */