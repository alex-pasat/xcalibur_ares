/**
 * @file robot_state.c
 * @brief Robot state machine implementation.
 */

#include "robot_state.h"
#include "drv8251.h"
#include "robot_control.h"
#include "robot_config.h"

#include "tiny_hsm.h"
#include "tiny_ring_buffer.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdint.h>
#include <string.h>

// -- Defines -----------------------------------------------------------------
// timeout for moves in milliseconds, used for watchdog timer
#define MOVE_TIMEOUT_MS 10000

#define FLAG_USING_GEOMETRY_DATA false

// -- PVs ---------------------------------------------------------------------

volatile tiny_hsm_t robot_hsm;

extern SPI_HandleTypeDef hspi1;

uint8_t spi_rx_raw;
uint8_t spi_tx_raw;

uint8_t spi_rx_store[SPI_BUF_SIZE];
tiny_ring_buffer_t spi_rx_buf;

uint8_t spi_tx_store[SPI_BUF_SIZE];
tiny_ring_buffer_t spi_tx_buf;

extern tiny_ring_buffer_t usb_rx_ring_buf;
extern usb_rx_packet_t usb_rx_packets[USB_RING_BUF_SIZE];

extern volatile uint16_t adc_dma_buf[];

// -- Motor Handles -----------------------------------------------------------

extern stepper_ctrl_t stepper_underpass;

extern motor_ctrl_t dc_pitch;
extern motor_ctrl_t dc_roll;
extern motor_ctrl_t dc_yaw;
extern motor_ctrl_t clamp;

// -- Knife Geometry ----------------------------------------------------------

const sharpening_parameters_t sharpening_params[N_KNIFE_TYPES] = {
  [KNIFETYPE_CHEF] = {.target_bevel_angle_deg = 5.0f},
  [KNIFETYPE_PARING] = {.target_bevel_angle_deg = 3.0f},
  [KNIFETYPE_GYOTO] = {.target_bevel_angle_deg = 2.0f},
  [KNIFETYPE_JAP_UTIL] = {.target_bevel_angle_deg = 5.0f},
};

static knife_type_t current_knife_type = KNIFETYPE_CHEF;

#if FLAG_USING_GEOMETRY_DATA
static uint8_t current_whetstone = 1;

#define ROLL_REF_OFFSET_DEG 0.0f // TODO
#define WHETSTONE_ONE_OFFSET() (ROLL_REF_OFFSET_DEG)
#define WHETSTONE_TWO_OFFSET() (ROLL_REF_OFFSET_DEG + 120.0f)
#define WHETSTONE_THREE_OFFSET() (ROLL_REF_OFFSET_DEG + 240.0f)

#define YAW_REF_OFFSET 0.0f // TODO
#define PITCH_REF_OFFSET 0.0f // TODO
#define UNDERPASS_OFFSET 0.0f
#endif

#define YAW_CONST_SPD_RPS 0.3f

static float roll_target_angle_deg = 0.0f;
static float pitch_target_angle_deg = 0.0f;
static float yaw_target_angle_deg = 0.0f;

static bool roll_approach_direction = 0;
static bool yaw_approach_direction = 0;
static bool pitch_approach_direction = 0;

// #define NUM_SHARPEN_PASSES 5
// static uint8_t sharpen_pass_count = 0;

static bool roll_move_complete = false;
static bool pitch_move_complete = false;
// static bool yaw_move_complete = false;
static bool underpass_move_complete = false;

// -- State Machine Configuration ---------------------------------------------

#if FLAG_USING_GEOMETRY_DATA
  #define X_GEO_DATA(id, fn, parent) X(id, fn, parent)
#else
  #define X_GEO_DATA(id, fn, parent)
#endif

#define ROBOT_STATE_TABLE(X) \
  X(STATE_TOP, state_top, tiny_hsm_no_parent) \
  X(STATE_AWAITING_KNIFE_SELECTION, state_await_knife_selection, state_top) \
  X(STATE_AWAITING_RPI_CONNECT, state_await_rpi_connect, state_top) \
  X(STATE_AWAITING_KNIFE_INPUT, state_await_knife_input, state_top) \
  X_GEO_DATA(STATE_RECV_HEADER, state_await_geometry_header, state_top) \
  X_GEO_DATA(STATE_RECV_YAW, state_await_geometry_yaw, state_top) \
  X_GEO_DATA(STATE_RECV_RATIOS1, state_await_geometry_ratios1, state_top) \
  X_GEO_DATA(STATE_RECV_RATIOS2, state_await_geometry_ratios2, state_top) \
  X(STATE_MOVING, state_moving, state_top) \
  X(STATE_HOMING_UNDERPASS, state_homing_underpass, state_moving) \
  X(STATE_MOCING_UNDERPASS, state_moving_underpass, state_moving) \
  X(STATE_HOMING_PITCH, state_homing_pitch, state_moving) \
  X(STATE_HOMING_ROLL, state_homing_roll, state_moving) \
  X(STATE_HOMING_YAW, state_homing_yaw, state_moving) \
  X(STATE_SHARPENING, state_sharpening, state_moving) \
  X(STATE_CLAMPING, state_clamping, state_moving) \
  X(STATE_UNCLAMPING, state_unclamping, state_moving) \
  X_GEO_DATA(STATE_PREPARE_YAW_A, state_prepare_yaw_a, state_moving) \
  X_GEO_DATA(STATE_PREPARE_ROLL_A, state_prepare_roll_a, state_moving) \
  X_GEO_DATA(STATE_PREPARE_UNDERPASS_A, state_prepare_underpass_a, state_moving) \
  X(STATE_SHARPEN_A, state_sharpen_a, state_moving) \
  X(STATE_ESTOP, state_estop, state_top)

#define X_ENUM(id, fn, parent) id,
typedef enum {
  ROBOT_STATE_TABLE(X_ENUM)
  STATE_INVALID = 0xFF,
} robot_state_id_t;
#undef X_ENUM

#define X_DECL(id, fn, parent) \
  static tiny_hsm_result_t fn(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
ROBOT_STATE_TABLE(X_DECL)
#undef X_DECL

#define X_DESCRIPTOR(id, fn, parent) \
  {(tiny_hsm_state_t)fn, (tiny_hsm_state_t)parent},
static const tiny_hsm_state_descriptor_t robot_hsm_states[] = {
  ROBOT_STATE_TABLE(X_DESCRIPTOR)
};
#undef X_DESCRIPTOR

static const tiny_hsm_configuration_t robot_hsm_config = {
  .states      = robot_hsm_states,
  .state_count = sizeof(robot_hsm_states) / sizeof(tiny_hsm_state_descriptor_t),
};

#define X_MAP(id, fn, parent) \
  if (robot_hsm.current == (tiny_hsm_state_t)fn) return id;
static robot_state_id_t get_current_state(void) {
  ROBOT_STATE_TABLE(X_MAP)
  return STATE_INVALID;
}
#undef X_MAP

static uint32_t global_tick_count = 0;

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

// -- AWAITING KNIFE SELECTION ------------------------------------------------

static tiny_hsm_result_t state_await_knife_selection(tiny_hsm_t *hsm,
                                             tiny_hsm_signal_t signal,
                                             const void *data) {
  switch (signal) {
  case tiny_hsm_signal_entry:
    LED_SetDuty(&led_strip, 1.0f);

    // clear SPI buffer to prepare for new command
    tiny_ring_buffer_clear(&spi_rx_buf);
    return tiny_hsm_result_signal_consumed;

  case tiny_hsm_signal_exit:
    // clear the output buffer so old data isn't sent to HMI
    tiny_ring_buffer_clear(&spi_tx_buf);
    return tiny_hsm_result_signal_consumed;

  case SIG_RECEIVED_SPI:
    if (data == NULL) {
      return tiny_hsm_result_signal_consumed;
    }
    hmi_to_robot_command_t cmd = *(uint8_t*)data;

    switch (cmd) {
      case KNIFETYPE_CHEF:
      case KNIFETYPE_PARING:
      case KNIFETYPE_GYOTO:
      case KNIFETYPE_JAP_UTIL:
        // valid knife type received, set knife type and transition to next state
        current_knife_type = (knife_type_t)cmd;
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
        break;

      case REQUESTING_DATA:
        uint8_t response = ROBOT_HMI_CMD_RPI_DETECTED;
        tiny_ring_buffer_insert(&spi_tx_buf, &response);
        return tiny_hsm_result_signal_consumed;

      default:
        // unhandled command
        return tiny_hsm_result_signal_consumed;
    }

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

// -- RPI REQUESTS ------------------------------------------------------------

static tiny_hsm_result_t state_await_rpi_connect(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_RECEIVED_USB:
      usb_rx_packet_t *pkt = (usb_rx_packet_t*)data;
      if (pkt == NULL || pkt->len == 0) {
        return tiny_hsm_result_signal_consumed;
      }

      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_selection);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_await_knife_input(tiny_hsm_t *hsm,
                                                     tiny_hsm_signal_t signal,
                                                     const void *data) {
  switch (signal) {
  case tiny_hsm_signal_entry:
    // send start signal to RPI
    USB_SendString("START\n");
    // clear USB buffer to prepare for new command
    tiny_ring_buffer_clear(&usb_rx_ring_buf);
    return tiny_hsm_result_signal_consumed;

  case tiny_hsm_signal_exit:
    return tiny_hsm_result_signal_consumed;

  case SIG_RECEIVED_USB:
    usb_rx_packet_t *pkt = (usb_rx_packet_t*)data;
    if (pkt == NULL || pkt->len == 0) {
      return tiny_hsm_result_signal_consumed;
    }

    if (strncmp((char*)pkt->buf, "FAIL", pkt->len) == 0) {
      uint8_t hmi_cmd = ROBOT_HMI_CMD_KNIFE_NOT_DETECTED;
      tiny_ring_buffer_insert(&spi_tx_buf, &hmi_cmd);
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
      return tiny_hsm_result_signal_consumed;
    }

    if (strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
      return tiny_hsm_result_signal_consumed;
    }

    if (strncmp((char*)pkt->buf, "KNIFE_DETECTED", pkt->len) == 0) {
#if FLAG_USING_GEOMETRY_DATA
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_geometry_header);
#else
      // send clamped data to HMI so it can display correct knife type
      uint8_t hmi_cmd = ROBOT_HMI_CMD_KNIFE_CLAMPED;
      tiny_ring_buffer_insert(&spi_tx_buf, &hmi_cmd);
      // tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_moving_underpass);
      // not moving the motors
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_sharpening);
#endif
    } 
   
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

// -- RPI GEOMETRY RECEPTION --------------------------------------------------

#if FLAG_USING_GEOMETRY_DATA
#define MAX_N  256

#define TIP_UNDERPASS_IDX 0
#define TIP_PITCH_IDX 1
#define TIP_yaw_angle_idx 2
#define TIP_ROLL_IDX 3

#define RATIO_UNDERPASS_IDX 0
#define RATIO_PITCH_IDX 1
#define RATIO_ROLL_IDX 2

typedef struct __attribute__((packed)) {
  double tip_q1[5];
  double tip_q2[5];
  uint16_t N;
} knife_header_t;

typedef struct {
  knife_header_t header;
  double    yaw_indices[MAX_N];
  double    ratios1[MAX_N-1][4];
  double    ratios2[MAX_N-1][4];
  bool ready;
} knife_profile_t;

typedef struct {
  uint8_t *write_ptr;
  uint32_t bytes_remaining;
  knife_profile_t profile;
} knife_parser_t;

static knife_parser_t knife_parser;
static uint8_t yaw_angle_idx;

static void feed_bytes(knife_parser_t *p, const uint8_t *buf, uint16_t len) {
  uint32_t to_copy = MIN(p->bytes_remaining, (uint32_t)len);
  memcpy(p->write_ptr, buf, to_copy);
  p->write_ptr       += to_copy;
  p->bytes_remaining -= to_copy;
}

static tiny_hsm_result_t state_await_geometry_header(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      knife_parser.write_ptr       = (uint8_t*)&knife_parser.profile.header;
      knife_parser.bytes_remaining = sizeof(knife_header_t);
      USB_SendString("START_VISION\n");
      tiny_ring_buffer_clear(&usb_rx_ring_buf);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_RECEIVED_USB:
      usb_rx_packet_t *pkt = (usb_rx_packet_t*)data;
      if (pkt == NULL || pkt->len == 0) {
        return tiny_hsm_result_signal_consumed;
      }

      if (strncmp((char*)pkt->buf, "FAIL", pkt->len) == 0) {
        uint8_t hmi_cmd = ROBOT_HMI_CMD_KNIFE_NOT_DETECTED;
        tiny_ring_buffer_insert(&spi_tx_buf, &hmi_cmd);
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_selection);
        return tiny_hsm_result_signal_consumed;
      }

      if (strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
        return tiny_hsm_result_signal_consumed;
      }

      feed_bytes(&knife_parser, pkt->buf, pkt->len);

      if (knife_parser.bytes_remaining == 0) {
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_geometry_yaw);
      }
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_await_geometry_yaw(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      // N is now known from the header we just parsed
      knife_parser.write_ptr       = (uint8_t*)knife_parser.profile.yaw_indices;
      knife_parser.bytes_remaining = knife_parser.profile.header.N * sizeof(double);
      // knife_parser.bytes_remaining = 165 * sizeof(double); // for testing, expect 165 yaw indices
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_RECEIVED_USB:
      usb_rx_packet_t *pkt = (usb_rx_packet_t*)data;
      if (pkt == NULL || pkt->len == 0) {
        return tiny_hsm_result_signal_consumed;
      }
      
      if (strncmp((char*)pkt->buf, "FAIL", pkt->len) == 0) {
        uint8_t hmi_cmd = ROBOT_HMI_CMD_KNIFE_NOT_DETECTED;
        tiny_ring_buffer_insert(&spi_tx_buf, &hmi_cmd);
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_selection);
        return tiny_hsm_result_signal_consumed;
      }

      if (strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
        return tiny_hsm_result_signal_consumed;
      }

      feed_bytes(&knife_parser, pkt->buf, pkt->len);

      if (knife_parser.bytes_remaining == 0) {
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_geometry_ratios1);
      }
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_await_geometry_ratios1(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      // N is now known from the header we just parsed
      knife_parser.write_ptr       = (uint8_t*)knife_parser.profile.ratios1;
      knife_parser.bytes_remaining = (knife_parser.profile.header.N - 1) * 4 * sizeof(double);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_RECEIVED_USB:
      usb_rx_packet_t *pkt = (usb_rx_packet_t*)data;
      if (pkt == NULL || pkt->len == 0) {
        return tiny_hsm_result_signal_consumed;
      }
      if (strncmp((char*)pkt->buf, "FAIL", pkt->len) == 0) {
        uint8_t hmi_cmd = ROBOT_HMI_CMD_KNIFE_NOT_DETECTED;
        tiny_ring_buffer_insert(&spi_tx_buf, &hmi_cmd);
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_selection);
        return tiny_hsm_result_signal_consumed;
      }

      if (strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
        return tiny_hsm_result_signal_consumed;
      }

      feed_bytes(&knife_parser, pkt->buf, pkt->len);

      if (knife_parser.bytes_remaining == 0) {
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_geometry_ratios2);
      }
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_await_geometry_ratios2(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      // N is now known from the header we just parsed
      knife_parser.write_ptr       = (uint8_t*)knife_parser.profile.ratios2;
      knife_parser.bytes_remaining = (knife_parser.profile.header.N - 1) * 4 * sizeof(double);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_RECEIVED_USB:
      usb_rx_packet_t *pkt = (usb_rx_packet_t*)data;
      if (pkt == NULL || pkt->len == 0) {
        return tiny_hsm_result_signal_consumed;
      }
      
      if (strncmp((char*)pkt->buf, "FAIL", pkt->len) == 0) {
        uint8_t hmi_cmd = ROBOT_HMI_CMD_KNIFE_NOT_DETECTED;
        tiny_ring_buffer_insert(&spi_tx_buf, &hmi_cmd);
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_selection);
        return tiny_hsm_result_signal_consumed;
      }

      if (strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
        return tiny_hsm_result_signal_consumed;
      }

      feed_bytes(&knife_parser, pkt->buf, pkt->len);

      if (knife_parser.bytes_remaining == 0) {
        USB_SendString("GEOMETRY_DATA_RECEIVED\n");

        knife_parser.profile.ready = true;
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_prepare_roll_a);
      }
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}
#endif

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

static tiny_hsm_result_t state_homing_underpass(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      StepperCtrl_StartHoming(&stepper_underpass);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      StepperCtrl_SetHome(&stepper_underpass);
      // tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_homing_roll);
      // dont home roll when yaw actuator broken
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_rpi_connect);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_moving_underpass(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      StepperCtrl_SetTarget(&stepper_underpass, 3000);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      uint8_t hmi_cmd = ROBOT_HMI_CMD_KNIFE_DONE;
      tiny_ring_buffer_insert(&spi_tx_buf, &hmi_cmd);

      StepperCtrl_SetTarget(&stepper_underpass, 0);
      tiny_hsm_transition(hsm, (tiny_hsm_state_t) state_await_knife_selection);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_homing_pitch(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      MotorCtrl_StartHoming(&dc_pitch);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      MotorCtrl_SetHome(&dc_pitch);
      MotorCtrl_ReEnableLimits(&dc_pitch);
      MotorCtrl_Stop(&dc_pitch);
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_homing_underpass);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_homing_roll(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      MotorCtrl_StartHoming(&dc_roll);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      MotorCtrl_SetHome(&dc_roll);
      MotorCtrl_Stop(&dc_roll);
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_rpi_connect);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_homing_yaw(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      MotorCtrl_StartHoming(&dc_yaw);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      MotorCtrl_SetHome(&dc_yaw);
      // drive yaw high so it doesn't slip
      DRV8251_Brake(dc_yaw.drv);
      MotorCtrl_ReEnableLimits(&dc_yaw);
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_homing_roll);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_sharpening(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_TICK:
      global_tick_count++;
      LED_PulseUpdate(&led_strip);
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      // send KNIFE_REMOVED command to HMI over SPI
      uint8_t response = ROBOT_HMI_CMD_KNIFE_DONE;
      tiny_ring_buffer_insert(&spi_tx_buf, &response);
      LED_SetDuty(&led_strip, 1.0f);
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_unclamping);
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
      uint8_t response = ROBOT_HMI_CMD_KNIFE_CLAMPED;
      tiny_ring_buffer_insert(&spi_tx_buf, &response);

#if FLAG_USING_GEOMETRY_DATA
      // send signal to USB task to notify knife is clamped and kinematics data can be sent
      // send here or is USB task?
      // USB_SendString((const char*)ROBOT_RPI_REQUEST_KINEMATICS);
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_geometry_header);
#else
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_sharpen_a);
#endif
      return tiny_hsm_result_signal_consumed;
    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_unclamping(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_TICK:
      global_tick_count++;
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      // send KNIFE_REMOVED command to HMI over SPI
      uint8_t response = ROBOT_HMI_CMD_KNIFE_REMOVED;
      tiny_ring_buffer_insert(&spi_tx_buf, &response);
      // transition back to awaiting knife selection state
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_selection);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

#if FLAG_USING_GEOMETRY_DATA
static tiny_hsm_result_t state_prepare_roll_a(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      switch (current_whetstone) {
        case 1: roll_target_angle_deg = knife_parser.profile.header.tip_q1[TIP_ROLL_IDX] + WHETSTONE_ONE_OFFSET(); break;
        case 2: roll_target_angle_deg = knife_parser.profile.header.tip_q1[TIP_ROLL_IDX] + WHETSTONE_TWO_OFFSET(); break;
        case 3: roll_target_angle_deg = knife_parser.profile.header.tip_q1[TIP_ROLL_IDX] + WHETSTONE_THREE_OFFSET(); break;
        default: break;
      }

      float current = MotorCtrl_GetCurrentAngleDeg(&dc_roll);
      roll_approach_direction = (roll_target_angle_deg > current);
      MotorCtrl_SetTarget(&dc_roll, dc_roll.homing_speed_rps);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      MotorCtrl_Stop(&dc_roll);
       tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_prepare_yaw_a);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_prepare_yaw_a(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      yaw_target_angle_deg = knife_parser.profile.header.tip_q1[TIP_yaw_angle_idx] + YAW_REF_OFFSET;
      float current = MotorCtrl_GetCurrentAngleDeg(&dc_yaw);
      yaw_approach_direction = (yaw_target_angle_deg > current);
      MotorCtrl_SetTarget(&dc_yaw, YAW_CONST_SPD_RPS);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      MotorCtrl_Stop(&dc_yaw);
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_prepare_pitch_a);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_prepare_pitch_a(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      pitch_target_angle_deg = knife_parser.profile.header.tip_q1[TIP_PITCH_IDX] + PITCH_REF_OFFSET;
      float current = MotorCtrl_GetCurrentAngleDeg(&dc_pitch);
      pitch_approach_direction = (pitch_target_angle_deg > current);
      MotorCtrl_SetTarget(&dc_pitch, dc_pitch.homing_speed_rps);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_prepare_underpass_a);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

static tiny_hsm_result_t state_prepare_underpass_a(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
    case tiny_hsm_signal_entry:
      float target_pos = knife_parser.profile.header.tip_q1[TIP_UNDERPASS_IDX] + UNDERPASS_OFFSET;
      StepperCtrl_SetTarget_m(&stepper_underpass, target_pos);
      return tiny_hsm_result_signal_consumed;

    case tiny_hsm_signal_exit:
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_sharpen_a);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
}
#endif

static tiny_hsm_result_t state_sharpen_a(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
  switch (signal) {
#if FLAG_USING_GEOMETRY_DATA
    case tiny_hsm_signal_entry:
      yaw_angle_idx = 0;
      yaw_approach_direction = (knife_parser.profile.yaw_indices[yaw_angle_idx] > MotorCtrl_GetCurrentAngleDeg(&dc_yaw));
      const_yaw_speed = yaw_approach_direction ? YAW_CONST_SPD_RPS : -YAW_CONST_SPD_RPS;
      // Set the constant yaw speed
      MotorCtrl_SetTarget(&dc_yaw, const_yaw_speed);

      // set the first target for roll and pitch based on the first set of ratios and the current yaw angle
      float ratio_pitch = knife_parser.profile.ratios1[yaw_angle_idx][RATIO_PITCH_IDX];
      float ratio_roll = knife_parser.profile.ratios1[yaw_angle_idx][RATIO_ROLL_IDX];
      float pitch_target_rps = ratio_pitch * const_yaw_speed;
      float roll_target_rps = ratio_roll * const_yaw_speed;

      MotorCtrl_SetTarget(&dc_pitch, pitch_target_rps);
      MotorCtrl_SetTarget(&dc_roll, roll_target_rps);

      // step the underpass based on the value in the profile and the current yaw angle
      float underpass_target_m = knife_parser.profile.ratios1[yaw_angle_idx][RATIO_UNDERPASS_IDX];
      StepperCtrl_SetTarget_m(&stepper_underpass, underpass_target_m);
      return tiny_hsm_result_signal_consumed;
#else 
    case tiny_hsm_signal_entry:
      roll_move_complete = false;
      pitch_move_complete = false;
      underpass_move_complete = false;

      pitch_target_angle_deg = 10.0f;
      pitch_approach_direction = (pitch_target_angle_deg > MotorCtrl_GetCurrentAngleDeg(&dc_pitch));
      
      MotorCtrl_SetTarget(&dc_roll, 1.0f);
      MotorCtrl_SetTarget(&dc_pitch, -dc_pitch.homing_speed_rps);
      StepperCtrl_SetTarget(&stepper_underpass, 300);
      return tiny_hsm_result_signal_consumed;
#endif

    case tiny_hsm_signal_exit:
      MotorCtrl_Stop(&dc_yaw);
      MotorCtrl_Stop(&dc_pitch);
      MotorCtrl_Stop(&dc_roll);
      StepperCtrl_Stop(&stepper_underpass);
      return tiny_hsm_result_signal_consumed;

#if FLAG_USING_GEOMETRY_DATA
    case SIG_MOVE_COMPLETE:
      yaw_angle_idx++;
      if (yaw_angle_idx >= knife_parser.profile.header.N - 1) {
        // finished sharpening
        MotorCtrl_Stop(&dc_yaw);
        MotorCtrl_Stop(&dc_pitch);
        MotorCtrl_Stop(&dc_roll);
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_sharpen_back_a);
      } else {
        // set next targets based on the new yaw angle
        float ratio_pitch = knife_parser.profile.ratios1[yaw_angle_idx][RATIO_PITCH_IDX];
        float ratio_roll = knife_parser.profile.ratios1[yaw_angle_idx][RATIO_ROLL_IDX];
        float pitch_target_rps = ratio_pitch * const_yaw_speed;
        float roll_target_rps = ratio_roll * const_yaw_speed;
        MotorCtrl_SetTarget(&dc_pitch, pitch_target_rps);
        MotorCtrl_SetTarget(&dc_roll, roll_target_rps);

        // step the underpass based on the value in the profile and the current yaw angle
        float underpass_target_m = knife_parser.profile.ratios1[yaw_angle_idx][RATIO_UNDERPASS_IDX];
        StepperCtrl_SetTarget_m(&stepper_underpass, underpass_target_m);
      }
      return tiny_hsm_result_signal_consumed;
#else 
    case SIG_TICK:
      if (global_tick_count >= 6000) {
        global_tick_count = 0;
        roll_move_complete = true;
      }
      global_tick_count++;
      return tiny_hsm_result_signal_consumed;

    case SIG_MOVE_COMPLETE:
      if (data == NULL) {
        return tiny_hsm_result_signal_consumed;
      }

      if (data == &dc_roll) { roll_move_complete = true; }
      if (data == &dc_pitch) { pitch_move_complete = true; }
      if (data == &stepper_underpass) { underpass_move_complete = true; }

      if (roll_move_complete && pitch_move_complete && underpass_move_complete) {
        MotorCtrl_Stop(&dc_yaw);
        MotorCtrl_Stop(&dc_roll);
        MotorCtrl_Stop(&dc_pitch);
        StepperCtrl_Stop(&stepper_underpass);

        uint8_t response = ROBOT_HMI_CMD_KNIFE_DONE;
        tiny_ring_buffer_insert(&spi_tx_buf, &response);
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_homing_pitch); 
      }
      return tiny_hsm_result_signal_consumed;
#endif

    default:
      return tiny_hsm_result_signal_deferred;
  }
}

// -- FAULT / ESTOP STATE -----------------------------------------------------

// static tiny_hsm_result_t state_comm_fault(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data) {
//   switch (signal) {
//     case tiny_hsm_signal_entry:
//     case tiny_hsm_signal_exit:
//       return tiny_hsm_result_signal_consumed;

//     default:
//       return tiny_hsm_result_signal_deferred;
//   }
// }

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
  // Initialize ring buffers
  memset(&spi_rx_store, 0, sizeof(spi_rx_store));
  memset(&spi_tx_store, 0, sizeof(spi_tx_store));

  tiny_ring_buffer_init(&spi_rx_buf, spi_rx_store, sizeof(uint8_t), SPI_BUF_SIZE);
  tiny_ring_buffer_init(&spi_tx_buf, spi_tx_store, sizeof(uint8_t), SPI_BUF_SIZE);  

  spi_tx_raw = ROBOT_HMI_CMD_NONE;
  HAL_SPI_TransmitReceive_IT(&hspi1, &spi_tx_raw, &spi_rx_raw, 1);

  // tiny_hsm_init((tiny_hsm_t*)&robot_hsm, &robot_hsm_config, (tiny_hsm_state_t)state_homing_pitch);
  tiny_hsm_init((tiny_hsm_t*)&robot_hsm, &robot_hsm_config, (tiny_hsm_state_t)state_await_rpi_connect);
}

void RobotState_SendSignal(robot_signal_t sig, const void *data) {
  tiny_hsm_send_signal((tiny_hsm_t*)&robot_hsm, sig, data);
}

void RobotState_DecoderHighFreq(void) {
  switch (get_current_state()) {
    case STATE_HOMING_UNDERPASS:
#if FLAG_USING_GEOMETRY_DATA
    case STATE_PREPARE_UNDERPASS_A:
#endif
    case STATE_SHARPEN_A:
      // steppers are awesome
      if(!StepperCtrl_Run(&stepper_underpass)) {
        RobotState_SendSignal(SIG_MOVE_COMPLETE, &stepper_underpass);
      }
      break;

    default:
      break;
  }
}

void RobotState_Decoder(void) {
  switch (get_current_state()) {
  case STATE_TOP:
    break;

  case STATE_AWAITING_KNIFE_SELECTION:
    if (tiny_ring_buffer_count(&spi_rx_buf) > 0) {
      uint8_t byte;
      tiny_ring_buffer_remove(&spi_rx_buf, &byte);
      if (byte != ROBOT_HMI_CMD_NONE)
        RobotState_SendSignal(SIG_RECEIVED_SPI, &byte);
    }
    break;

  case STATE_AWAITING_RPI_CONNECT:
  case STATE_AWAITING_KNIFE_INPUT:
#if FLAG_USING_GEOMETRY_DATA
  case STATE_RECV_HEADER:
  case STATE_RECV_YAW:
  case STATE_RECV_RATIOS1:
  case STATE_RECV_RATIOS2:
#endif
    usb_rx_packet_t pkt;
    if (tiny_ring_buffer_count(&usb_rx_ring_buf) > 0) {
      tiny_ring_buffer_remove(&usb_rx_ring_buf, &pkt);
      RobotState_SendSignal(SIG_RECEIVED_USB, &pkt);
    }
    break;

  case STATE_HOMING_UNDERPASS:
     // handled in high freq decoder
    break;

  case STATE_HOMING_PITCH:
    if (dc_pitch.hall_triggered) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    }
    break;

  case STATE_HOMING_ROLL:
    if (dc_roll.hall_triggered) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    }
    break;

  case STATE_HOMING_YAW:
    if (dc_yaw.hall_triggered) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    }
    break;

  case STATE_CLAMPING:
    // check if the current is within threshold of target and if so,
    // send move complete signal and transition to idle
    if (dc_clamp.current_ma > CURRENT_THRESHOLD_KNIFECLAMP_MA) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    }
    break;

  case STATE_SHARPENING:
     if (global_tick_count >= 1500) { // 1500 * 10ms = 15s,
      global_tick_count = 0;
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    } else {
    RobotState_SendSignal(SIG_TICK, NULL);
    }
    break;

  case STATE_UNCLAMPING:
    if (global_tick_count >= 50) { // 50 * 10ms = 500ms,
      global_tick_count = 0;
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    } else {
      RobotState_SendSignal(SIG_TICK, NULL);
    }
    break;

#if FLAG_USING_GEOMETRY_DATA
  case STATE_PREPARE_ROLL_A:
    float current = MotorCtrl_GetCurrentAngleDeg(&dc_roll);
    bool crossed_target = roll_approach_direction ?
     (current >= roll_target_angle_deg) : (current <= roll_target_angle_deg);
    if (crossed_target) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    }
    break;

  case STATE_PREPARE_YAW_A:
    float current_yaw = MotorCtrl_GetCurrentAngleDeg(&dc_yaw);
    bool crossed_target_yaw = yaw_approach_direction ?
     (current_yaw >= yaw_target_angle_deg) : (current_yaw <= yaw_target_angle_deg);
    if (crossed_target_yaw) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    }
    break;

  case STATE_PREPARE_PITCH_A:
    float current_pitch = MotorCtrl_GetCurrentAngleDeg(&dc_pitch);
    bool crossed_target_pitch = pitch_approach_direction ?
     (current_pitch >= pitch_target_angle_deg) : (current_pitch <= pitch_target_angle_deg);
    if (crossed_target_pitch) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    }
    break;

  case STATE_PREPARE_UNDERPASS_A:
    // handled in high freq decoder
    break;
#endif

  case STATE_SHARPEN_A:
#if FLAG_USING_GEOMETRY_DATA
    /*
    compare the yaw angle to the current target yaw index in the knife
    profile, and if we have reached or passed the target, update the target
    angles for roll, pitch, and underpass based on the ratios in the knife
    */
    float current_sharpen_yaw = MotorCtrl_GetCurrentAngleDeg(&dc_yaw);
    float next_yaw_target = (float)knife_parser.profile.yaw_indices[yaw_angle_idx];
    bool crossed = yaw_approach_direction ?
      (current_sharpen_yaw >= next_yaw_target) :
      (current_sharpen_yaw <= next_yaw_target);
    if (crossed) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, NULL);
    }
    break;
#else
    LED_PulseUpdate(&led_strip);

    // check pitch
    float current_sharpen_pitch = MotorCtrl_GetCurrentAngleDeg(&dc_pitch);
    bool crossed_target_pitch = pitch_approach_direction ?
      (current_sharpen_pitch >= pitch_target_angle_deg) :
      (current_sharpen_pitch <= pitch_target_angle_deg);

    if (crossed_target_pitch && !pitch_move_complete) {
      RobotState_SendSignal(SIG_MOVE_COMPLETE, &dc_pitch);
    }
    break;

#endif
    
  case STATE_ESTOP: break;

  default:
    break;
  }
}

// -- HAL Callbacks -----------------------------------------------------------

#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
  if (h->Instance == USART1) {
    tiny_ring_buffer_insert(&usb_buf, usb_rx_buf);
    HAL_UART_Receive_IT(h, usb_rx_buf, 1);
  }
}
#endif