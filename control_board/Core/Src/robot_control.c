/**
 * @file robot_control.c
 * @brief Robot control state machine implementation.
 */
#include "robot_control.h"
#include "robot_config.h"

// -- Defines -----------------------------------------------------------------
#define MOVE_TIMEOUT_MS 10000 // timeout for moves in milliseconds, used for watchdog timer

// -- PVs ---------------------------------------------------------------------

static uint32_t watchdog_ms = 0; // used to track time spent in moving state for watchdog timer

// SPI buffer for communicating with motor drivers, etc.
static uint8_t spi_buffer[256];

// -- TOP STATE --------------------------------------------------------------

tiny_hsm_result_t state_top(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
    switch (signal) {
        case SIG_CMD_ESTOP:
            // TODO: handle emergency stop command

            tiny_hsm_transition(hsm, (tiny_hsm_state_t)ROBOT_STATE_ESTOP);
            return tiny_hsm_result_signal_consumed;

        default:
            return tiny_hsm_result_signal_deferred;
    }
}

// -- IDLE STATE --------------------------------------------------------------

tiny_hsm_result_t state_idle(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
    switch (signal) {
        case tiny_hsm_signal_entry:
            return tiny_hsm_result_signal_consumed;

        case tiny_hsm_signal_exit:
            return tiny_hsm_result_signal_consumed;

        case SIG_USER_INPUT:
            // TODO: handle user input
            tiny_hsm_transition(hsm, (tiny_hsm_state_t)ROBOT_STATE_USER_INPUT);
            return tiny_hsm_result_signal_consumed;

        case SIG_CAMERA_INPUT:
            // TODO: handle camera input
            tiny_hsm_transition(hsm, (tiny_hsm_state_t)ROBOT_STATE_CAMERA);
            return tiny_hsm_result_signal_consumed;

        default:
            return tiny_hsm_result_signal_deferred;
    }
}

tiny_hsm_result_t state_user_input(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
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

tiny_hsm_result_t state_operational(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
    switch (signal) {
        case tiny_hsm_signal_entry:
            // runs once when entering the operational state
            return tiny_hsm_result_signal_consumed;

        case tiny_hsm_signal_exit:
            // runs once when exiting the operational state
            return tiny_hsm_result_signal_consumed;

        case SIG_FAULT:
            tiny_hsm_transition(hsm, (tiny_hsm_state_t)ROBOT_STATE_FAULT);
            return tiny_hsm_result_signal_consumed;

        default:
            return tiny_hsm_result_signal_deferred;
    }
}

tiny_hsm_result_t state_moving(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
    switch (signal) {
        case tiny_hsm_signal_entry:
            return tiny_hsm_result_signal_consumed;

        case tiny_hsm_signal_exit:
            watchdog_ms = 0; // reset watchdog on exit from moving state
            return tiny_hsm_result_signal_consumed;

        case SIG_MOVE_COMPLETE:
            if (/* TODO: check if more moves are queued */0) {
                // pop next move from queue and execute
                // tiny_hsm_send_signal(hsm, q, NULL);
            } else {
                tiny_hsm_transition(hsm, (tiny_hsm_state_t)ROBOT_STATE_IDLE);
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

tiny_hsm_result_t state_mv_home(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
    if (signal == tiny_hsm_signal_entry) {
        // TODO: start homing sequence

        watchdog_ms = MOVE_TIMEOUT_MS;
        return tiny_hsm_result_signal_consumed;
    }
    return tiny_hsm_result_signal_deferred;
}

// -- FAULT / ESTOP STATE -----------------------------------------------------

tiny_hsm_result_t state_fault(tiny_hsm_t *hsm, tiny_hsm_signal_t sig, const void *data) {
    switch (sig) {
        case tiny_hsm_signal_entry:
            // TODO: handle fault entry (e.g. stop all motors, disable outputs, etc.)
            return tiny_hsm_result_signal_consumed;
        case tiny_hsm_signal_exit:
            // TODO: handle fault exit (e.g. re-enable outputs, etc.)
            return tiny_hsm_result_signal_consumed;

        case SIG_FAULT_CLEARED:
            // TODO: handle fault cleared (e.g. check if its safe to resume operation, etc.)
            tiny_hsm_transition(hsm, (tiny_hsm_state_t)ROBOT_STATE_USER_INPUT);
            return tiny_hsm_result_signal_consumed;

        default:
            return tiny_hsm_result_signal_deferred;
    }
}

tiny_hsm_result_t state_estop(tiny_hsm_t *hsm, tiny_hsm_signal_t sig, const void *data) {
    switch (sig) {
        case tiny_hsm_signal_entry:
            // TODO: handle estop entry (e.g. stop all motors immediately, disable outputs, etc.)
            return tiny_hsm_result_signal_consumed;
        case tiny_hsm_signal_exit:
            // TODO: handle estop exit (e.g. re-enable outputs, etc.)
            return tiny_hsm_result_signal_consumed;

        default:
            return tiny_hsm_result_signal_deferred;
    }
}