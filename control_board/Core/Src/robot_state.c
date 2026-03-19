/**
 * @file robot_state.c
 * @brief Robot state machine implementation.
 */

#include "robot_state.h"
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
  [KNIFETYPE_CHEF] = {.target_bevel_angle_deg = 20.0f},
  [KNIFETYPE_PARING] = {.target_bevel_angle_deg = 15.0f},
  [KNIFETYPE_GYOTO] = {.target_bevel_angle_deg = 18.0f},
  [KNIFETYPE_JAP_UTIL] = {.target_bevel_angle_deg = 10.0f},
  // TODO: set thse to actual sharpening params
};

static knife_type_t current_knife_type = KNIFETYPE_CHEF;

// -- Function Prototypes -----------------------------------------------------

typedef enum {
  STATE_TOP,

  STATE_AWAITING_KNIFE_SELECTION,
  STATE_AWAITING_RPI_CONNECT,
  STATE_AWAITING_KNIFE_INPUT,
  STATE_RECV_HEADER,
  STATE_RECV_YAW,
  STATE_RECV_RATIOS1,
  STATE_RECV_RATIOS2,

  STATE_MOVING,
  STATE_HOMING,

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

static tiny_hsm_result_t state_await_knife_selection(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);

static tiny_hsm_result_t state_await_rpi_connect(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_await_knife_input(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_await_geometry_header(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_await_geometry_yaw(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_await_geometry_ratios1(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);
static tiny_hsm_result_t state_await_geometry_ratios2(tiny_hsm_t *hsm, tiny_hsm_signal_t signal, const void *data);

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

  if (robot_hsm.current == (tiny_hsm_state_t)state_await_knife_selection) return STATE_AWAITING_KNIFE_SELECTION;
  if (robot_hsm.current == (tiny_hsm_state_t)state_await_rpi_connect) return STATE_AWAITING_RPI_CONNECT;
  if (robot_hsm.current == (tiny_hsm_state_t)state_await_knife_input) return STATE_AWAITING_KNIFE_INPUT;
  if (robot_hsm.current == (tiny_hsm_state_t)state_await_geometry_header) return STATE_RECV_HEADER;
  if (robot_hsm.current == (tiny_hsm_state_t)state_await_geometry_yaw) return STATE_RECV_YAW;
  if (robot_hsm.current == (tiny_hsm_state_t)state_await_geometry_ratios1) return STATE_RECV_RATIOS1;
  if (robot_hsm.current == (tiny_hsm_state_t)state_await_geometry_ratios2) return STATE_RECV_RATIOS2;
  
  if (robot_hsm.current == (tiny_hsm_state_t)state_moving) return STATE_MOVING;
  if (robot_hsm.current == (tiny_hsm_state_t)state_homing) return STATE_HOMING;
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
    
    {.state = (tiny_hsm_state_t)state_await_knife_selection, .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_await_rpi_connect, .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_await_knife_input, .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_await_geometry_header, .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_await_geometry_yaw, .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_await_geometry_ratios1, .parent = (tiny_hsm_state_t)state_top},
    {.state = (tiny_hsm_state_t)state_await_geometry_ratios2, .parent = (tiny_hsm_state_t)state_top},

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

// -- AWAITING KNIFE SELECTION ------------------------------------------------

static tiny_hsm_result_t state_await_knife_selection(tiny_hsm_t *hsm,
                                             tiny_hsm_signal_t signal,
                                             const void *data) {
  switch (signal) {
  case tiny_hsm_signal_entry:
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
        // tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_clamping);
        break;

      case REQUESTING_DATA:
        uint8_t response = ROBOT_HMI_CMD_RPI_DETECTED;
        tiny_ring_buffer_insert(&spi_tx_buf, &response);
        return tiny_hsm_result_signal_consumed;

      default:
        // invalid command, ignore
        return tiny_hsm_result_signal_consumed;
    }

    return tiny_hsm_result_signal_consumed;

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

      // transition to await knife input
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);

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
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
      return tiny_hsm_result_signal_consumed;
    }

    if (strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
      // TODO: change this to go to different state (maybe add error state?) and work with HMI
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
      return tiny_hsm_result_signal_consumed;
    }

    if (strncmp((char*)pkt->buf, "KNIFE_DETECTED", pkt->len) == 0) {
      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_geometry_header);
    } 
   
    return tiny_hsm_result_signal_consumed;

  default:
    return tiny_hsm_result_signal_deferred;
  }
}

#define MAX_N  256

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

      if (strncmp((char*)pkt->buf, "FAIL", pkt->len) == 0 ||
        strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
        // TODO: change this to go to different state (maybe add error state?) and work with HMI
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
      if (strncmp((char*)pkt->buf, "FAIL", pkt->len) == 0 ||
          strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
      // TODO: change this to go to different state (maybe add error state?) and work with HMI
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
      if (strncmp((char*)pkt->buf, "FAIL", pkt->len) == 0 ||
          strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
      // TODO: change this to go to different state (maybe add error state?) and work with HMI
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
      if (strncmp((char*)pkt->buf, "FAIL", pkt->len) == 0 ||
          strncmp((char*)pkt->buf, "RECONNECTED", pkt->len) == 0) {
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
        return tiny_hsm_result_signal_consumed;
      }

      feed_bytes(&knife_parser, pkt->buf, pkt->len);

      if (knife_parser.bytes_remaining == 0) {
        USB_SendString("GEOMETRY_DATA_RECEIVED\n");

        knife_parser.profile.ready = true;
        tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_input);
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
    // for testing
    DRV8251_SetSpeed(dc_roll.drv, 0.5f);
    return tiny_hsm_result_signal_consumed;

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

    // TODO: await move complete from all homing motors

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
      // USB_SendString((const char*)ROBOT_RPI_REQUEST_KINEMATICS);

      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_geometry_header);
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

    case SIG_MOVE_COMPLETE:
      // send KNIFE_REMOVED command to HMI over SPI
      tiny_ring_buffer_insert(
        &spi_tx_buf, (uint8_t*)ROBOT_HMI_CMD_KNIFE_REMOVED);

      tiny_hsm_transition(hsm, (tiny_hsm_state_t)state_await_knife_selection);
      return tiny_hsm_result_signal_consumed;

    default:
      return tiny_hsm_result_signal_deferred;
  }
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

  tiny_hsm_init((tiny_hsm_t*)&robot_hsm, &robot_hsm_config, (tiny_hsm_state_t)state_await_knife_selection);
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
    if (tiny_ring_buffer_count(&spi_rx_buf) > 0) {
      uint8_t byte;
      tiny_ring_buffer_remove(&spi_rx_buf, &byte);
      if (byte != ROBOT_HMI_CMD_NONE)
        RobotState_SendSignal(SIG_RECEIVED_SPI, &byte);
    }
    break;

  case STATE_AWAITING_RPI_CONNECT:
  case STATE_AWAITING_KNIFE_INPUT:
  case STATE_RECV_HEADER:
  case STATE_RECV_YAW:
  case STATE_RECV_RATIOS1:
  case STATE_RECV_RATIOS2:
    usb_rx_packet_t pkt;
    if (tiny_ring_buffer_count(&usb_rx_ring_buf) > 0) {
      tiny_ring_buffer_remove(&usb_rx_ring_buf, &pkt);
      RobotState_SendSignal(SIG_RECEIVED_USB, &pkt);
    }
    break;

  case STATE_HOMING:
    // TODO: finish homing procedure

    if (stepper_underpass.limit_triggered) {
      StepperCtrl_Stop(&stepper_underpass);
      StepperCtrl_SetHome(&stepper_underpass);
    }

    // bring clamp to limit switch? 
    if (clamp.limit_triggered) {
      MotorCtrl_Stop(&clamp);
    }

    // bring pitch, roll, yaw to home position hall effect
    if (dc_pitch.hall_triggered) {
      MotorCtrl_Stop(&dc_pitch);
    }
    if (dc_roll.hall_triggered) {
      MotorCtrl_Stop(&dc_roll);
    }
    if (dc_yaw.hall_triggered) {
      MotorCtrl_Stop(&dc_yaw);
    }

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
      tiny_hsm_transition((tiny_hsm_t*)&robot_hsm, (tiny_hsm_state_t)state_await_knife_selection);
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



#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
  if (h->Instance == USART1) {
    tiny_ring_buffer_insert(&usb_buf, usb_rx_buf);
    HAL_UART_Receive_IT(h, usb_rx_buf, 1);
  }
}
#endif