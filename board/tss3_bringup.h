#ifndef BOARD_TSS3_BRINGUP_H
#define BOARD_TSS3_BRINGUP_H

#include <stdbool.h>
#include <stdint.h>

#include "board/obj/tss3_request_signer_payload.h"
#include "board/tss3_aes.h"

#define TSS3_TX_ADDR 0x7A1U
#define TSS3_RX_ADDR 0x7A9U
#define TSS3_BUS 0U
#define TSS3_ENABLED_MAGIC 0x33535354U
#define TSS3_WAKE_MAGIC 0x57414B45U
#define TSS3_RX_QUEUE_SIZE 16U
#define TSS3_RESPONSE_TIMEOUT_US 1000000U
#define TSS3_PENDING_TIMEOUT_US 5000000U
#define TSS3_BOOT_DELAY_US 250000U
#define TSS3_BOOT_TIMEOUT_US 10000000U
#define TSS3_TX_DRAIN_US 100000U

typedef enum {
  TSS3_PHASE_IDLE = 0,
  TSS3_PHASE_APP_EXTENDED,
  TSS3_PHASE_APP_PROGRAMMING,
  TSS3_PHASE_BOOT_DELAY,
  TSS3_PHASE_BOOT_PROBE,
  TSS3_PHASE_BOOT_DEFAULT,
  TSS3_PHASE_BOOT_EXTENDED,
  TSS3_PHASE_BOOT_PROGRAMMING,
  TSS3_PHASE_REQUEST_SEED,
  TSS3_PHASE_SEND_KEY,
  TSS3_PHASE_WRITE_DID_0203,
  TSS3_PHASE_WRITE_DID_0201,
  TSS3_PHASE_WRITE_DID_0202,
  TSS3_PHASE_REQUEST_DOWNLOAD,
  TSS3_PHASE_TRANSFER_DATA,
  TSS3_PHASE_TRANSFER_EXIT,
  TSS3_PHASE_VERIFY,
  TSS3_PHASE_TRIGGER,
  TSS3_PHASE_TX_DRAIN,
} tss3_phase_t;

typedef enum { TSS3_TX_IDLE, TSS3_TX_WAIT_FC, TSS3_TX_SENDING, TSS3_TX_WAIT_REPLY } tss3_tx_state_t;

typedef struct {
  uint32_t enabled;
  uint32_t wake;
} tss3_retained_t;

typedef struct {
  uint8_t data[8];
} tss3_rx_frame_t;

typedef struct {
  volatile tss3_phase_t phase;
  volatile bool start_pending;
  volatile bool rx_overflow;
  tss3_tx_state_t tx_state;
  uint8_t transfer_block;
  uint8_t tx_prefix[20];
  uint8_t tx_prefix_length;
  const uint8_t *tx_data;
  uint16_t tx_length;
  uint16_t tx_offset;
  uint8_t tx_sequence;
  uint8_t tx_block_remaining;
  uint32_t tx_stmin_us;
  uint32_t tx_next_us;
  uint8_t rx_data[32];
  uint16_t rx_length;
  uint16_t rx_expected;
  uint8_t rx_sequence;
  uint32_t timer_deadline_us;
  uint32_t run_started_us;
  volatile uint8_t rx_read;
  volatile uint8_t rx_write;
  tss3_rx_frame_t rx_queue[TSS3_RX_QUEUE_SIZE];
} tss3_bringup_t;

__attribute__((section(".backup_sram"))) static volatile tss3_retained_t tss3_retained;
static tss3_bringup_t tss3_bringup;

static const uint8_t tss3_session_extended[] = {0x10U, 0x03U};

static bool tss3_bringup_enabled(void) {
  return tss3_retained.enabled == TSS3_ENABLED_MAGIC;
}

static void tss3_bringup_set_enabled(bool enabled) {
  tss3_retained.enabled = enabled ? TSS3_ENABLED_MAGIC : 0U;
  if (!enabled) {
    tss3_retained.wake = 0U;
  }
}

static bool tss3_bringup_active(void) {
  return tss3_bringup.phase != TSS3_PHASE_IDLE;
}

static void tss3_set_timer(uint32_t now, uint32_t duration) {
  tss3_bringup.timer_deadline_us = now + duration;
}

static bool tss3_timer_expired(uint32_t now) {
  return get_ts_elapsed(now, tss3_bringup.timer_deadline_us) < 0x80000000U;
}

static void tss3_send_can(const uint8_t *data) {
  CANPacket_t packet = {.addr = TSS3_TX_ADDR, .bus = TSS3_BUS, .data_len_code = 8U};
  (void)memcpy(packet.data, data, 8U);
  can_set_checksum(&packet);
  can_send(&packet, TSS3_BUS, false);
}

static void tss3_send_tx_frame(uint8_t *frame, uint8_t header_length) {
  uint16_t offset = tss3_bringup.tx_offset;
  for (uint8_t i = header_length; (i < 8U) && (offset < tss3_bringup.tx_length); i++) {
    frame[i] = (offset < tss3_bringup.tx_prefix_length) ? tss3_bringup.tx_prefix[offset] :
               tss3_bringup.tx_data[offset - tss3_bringup.tx_prefix_length];
    offset++;
  }
  tss3_bringup.tx_offset = offset;
  tss3_send_can(frame);
}

static void tss3_clear_pending_rx(void) {
  tss3_retained.wake = 0U;
  tss3_bringup.start_pending = false;
  tss3_bringup.rx_overflow = false;
  tss3_bringup.rx_read = tss3_bringup.rx_write;
}

static void tss3_finish(void) {
  tss3_bringup.phase = TSS3_PHASE_IDLE;
  tss3_clear_pending_rx();
  tss3_bringup.tx_state = TSS3_TX_IDLE;
  tss3_bringup.rx_expected = 0U;
  heartbeat_disabled = false;
  if (safety_setup.current_safety_mode == SAFETY_ELM327) {
    set_safety_mode(SAFETY_SILENT, 0U);
  }
}

static void tss3_tx_complete(uint32_t now) {
  tss3_bringup.tx_state = TSS3_TX_IDLE;
  if (tss3_bringup.phase == TSS3_PHASE_APP_PROGRAMMING) {
    tss3_bringup.phase = TSS3_PHASE_BOOT_DELAY;
    tss3_set_timer(now, TSS3_BOOT_DELAY_US);
  } else if (tss3_bringup.phase == TSS3_PHASE_TRIGGER) {
    tss3_bringup.phase = TSS3_PHASE_TX_DRAIN;
    tss3_set_timer(now, TSS3_TX_DRAIN_US);
  } else {
    tss3_bringup.tx_state = TSS3_TX_WAIT_REPLY;
    tss3_set_timer(now, TSS3_RESPONSE_TIMEOUT_US);
  }
}

static void tss3_start_request(const uint8_t *prefix, uint8_t prefix_length,
                               const uint8_t *data, uint16_t data_length, uint32_t now) {
  tss3_bringup.tx_prefix_length = prefix_length;
  (void)memcpy(tss3_bringup.tx_prefix, prefix, prefix_length);
  tss3_bringup.tx_data = data;
  tss3_bringup.tx_length = (uint16_t)prefix_length + data_length;
  tss3_bringup.tx_offset = 0U;
  tss3_bringup.tx_sequence = 1U;
  tss3_bringup.rx_expected = 0U;

  uint8_t frame[8] = {0};
  if (tss3_bringup.tx_length <= 7U) {
    frame[0] = (uint8_t)tss3_bringup.tx_length;
    tss3_send_tx_frame(frame, 1U);
    tss3_tx_complete(now);
  } else {
    frame[0] = 0x10U | (uint8_t)(tss3_bringup.tx_length >> 8);
    frame[1] = (uint8_t)tss3_bringup.tx_length;
    tss3_bringup.tx_state = TSS3_TX_WAIT_FC;
    tss3_send_tx_frame(frame, 2U);
    tss3_set_timer(now, TSS3_RESPONSE_TIMEOUT_US);
  }
}

static void tss3_send_session(const uint8_t *request, tss3_phase_t phase, uint32_t now) {
  tss3_bringup.phase = phase;
  tss3_start_request(request, 2U, (const uint8_t *)0, 0U, now);
}

static void tss3_send_transfer_block(uint32_t now) {
  const uint8_t prefix[2] = {0x36U, (uint8_t)(tss3_bringup.transfer_block + 1U)};
  tss3_bringup.phase = TSS3_PHASE_TRANSFER_DATA;
  tss3_start_request(prefix, 2U,
                     &tss3_request_signer_payload()[(uint32_t)tss3_bringup.transfer_block * 0x400U],
                     0x400U, now);
}

static void tss3_begin(uint32_t now) {
  tss3_clear_pending_rx();
  tss3_bringup.run_started_us = now;
  tss3_bringup.transfer_block = 0U;
  set_safety_mode(SAFETY_ELM327, 1U);
  heartbeat_disabled = true;
  tss3_send_session(tss3_session_extended, TSS3_PHASE_APP_EXTENDED, now);
}

static void tss3_send_key(const uint8_t *seed, uint32_t now) {
  uint8_t request[18] = {[0] = 0x27U, [1] = 0x02U};
  tss3_aes_encrypt(seed, &request[2]);
  tss3_bringup.phase = TSS3_PHASE_SEND_KEY;
  tss3_start_request(request, sizeof(request), (const uint8_t *)0, 0U, now);
}

static void tss3_handle_positive_reply(const uint8_t *reply, uint16_t length, uint32_t now) {
  static const uint8_t session_default[] = {0x10U, 0x01U};
  static const uint8_t session_programming[] = {0x10U, 0x02U};
  static const uint8_t request_seed[18] = {[0] = 0x27U, [1] = 0x01U};
  static const uint8_t write_did_0203[8] = {[0] = 0x2eU, [1] = 0x02U, [2] = 0x03U};
  static const uint8_t write_did_0201[19] = {[0] = 0x2eU, [1] = 0x02U, [2] = 0x01U};
  static const uint8_t write_did_0202[19] = {[0] = 0x2eU, [1] = 0x02U, [2] = 0x02U};
  static const uint8_t request_download[] = {
    0x34U, 0x01U, 0x46U, 0x01U, 0x00U, 0xfeU, 0xbfU, 0x00U, 0x00U,
    0x00U, 0x00U, 0x10U, 0x00U,
  };
  static const uint8_t transfer_exit[] = {0x37U};
  static const uint8_t verify[] = {
    0x31U, 0x01U, 0x10U, 0xf0U, 0x45U, 0x00U, 0xfeU, 0xbfU,
    0x00U, 0x00U, 0x00U, 0x00U, 0x10U, 0x00U,
  };
  static const uint8_t trigger[] = {
    0x31U, 0x01U, 0xffU, 0x00U, 0x45U, 0x00U, 0x00U, 0x0eU,
    0x00U, 0x00U, 0x00U, 0x00U, 0x80U, 0x00U,
  };
  static const struct {
    tss3_phase_t next_phase;
    const uint8_t *request;
    uint8_t request_length;
    uint8_t reply_length;
    uint8_t reply[3];
  } steps[] = {
    [TSS3_PHASE_APP_EXTENDED] = {TSS3_PHASE_APP_PROGRAMMING, session_programming, sizeof(session_programming), 1U, {[0] = 0x03U}},
    [TSS3_PHASE_BOOT_PROBE] = {TSS3_PHASE_BOOT_DEFAULT, session_default, sizeof(session_default), 1U, {[0] = 0x03U}},
    [TSS3_PHASE_BOOT_DEFAULT] = {TSS3_PHASE_BOOT_EXTENDED, tss3_session_extended, sizeof(tss3_session_extended), 1U, {[0] = 0x01U}},
    [TSS3_PHASE_BOOT_EXTENDED] = {TSS3_PHASE_BOOT_PROGRAMMING, session_programming, sizeof(session_programming), 1U, {[0] = 0x03U}},
    [TSS3_PHASE_BOOT_PROGRAMMING] = {TSS3_PHASE_REQUEST_SEED, request_seed, sizeof(request_seed), 1U, {[0] = 0x02U}},
    [TSS3_PHASE_SEND_KEY] = {TSS3_PHASE_WRITE_DID_0203, write_did_0203, sizeof(write_did_0203), 1U, {[0] = 0x02U}},
    [TSS3_PHASE_WRITE_DID_0203] = {TSS3_PHASE_WRITE_DID_0201, write_did_0201, sizeof(write_did_0201), 2U, {0x02U, 0x03U, 0U}},
    [TSS3_PHASE_WRITE_DID_0201] = {TSS3_PHASE_WRITE_DID_0202, write_did_0202, sizeof(write_did_0202), 2U, {0x02U, 0x01U, 0U}},
    [TSS3_PHASE_WRITE_DID_0202] = {TSS3_PHASE_REQUEST_DOWNLOAD, request_download, sizeof(request_download), 2U, {0x02U, 0x02U, 0U}},
    [TSS3_PHASE_TRANSFER_EXIT] = {TSS3_PHASE_VERIFY, verify, sizeof(verify), 0U, {0U, 0U, 0U}},
    [TSS3_PHASE_VERIFY] = {TSS3_PHASE_TRIGGER, trigger, sizeof(trigger), 3U, {0x01U, 0x10U, 0xf0U}},
  };

  const tss3_phase_t phase = tss3_bringup.phase;
  bool valid = false;
  if (((uint32_t)phase < (sizeof(steps) / sizeof(steps[0]))) && (steps[phase].request != (const uint8_t *)0)) {
    valid = (length >= (1U + steps[phase].reply_length)) &&
            (memcmp(&reply[1], steps[phase].reply, steps[phase].reply_length) == 0);
    if (valid) {
      tss3_bringup.phase = steps[phase].next_phase;
      tss3_start_request(steps[phase].request, steps[phase].request_length, (const uint8_t *)0, 0U, now);
    }
  } else {
    switch (phase) {
      case TSS3_PHASE_REQUEST_SEED:
        valid = (length == 18U) && (reply[1] == 0x01U);
        if (valid) {
          tss3_send_key(&reply[2], now);
        }
        break;
      case TSS3_PHASE_REQUEST_DOWNLOAD:
        valid = true;
        tss3_bringup.transfer_block = 0U;
        tss3_send_transfer_block(now);
        break;
      case TSS3_PHASE_TRANSFER_DATA:
        valid = (length >= 2U) && (reply[1] == (uint8_t)(tss3_bringup.transfer_block + 1U));
        if (valid) {
          tss3_bringup.transfer_block += 1U;
          if (tss3_bringup.transfer_block < 4U) {
            tss3_send_transfer_block(now);
          } else {
            tss3_bringup.phase = TSS3_PHASE_TRANSFER_EXIT;
            tss3_start_request(transfer_exit, sizeof(transfer_exit), (const uint8_t *)0, 0U, now);
          }
        }
        break;
      default:
        break;
    }
  }
  if (!valid) {
    tss3_finish();
  }
}

static void tss3_handle_response(const uint8_t *reply, uint16_t length, uint32_t now) {
  if ((tss3_bringup.tx_state == TSS3_TX_WAIT_REPLY) && (length != 0U)) {
    const uint8_t sid = tss3_bringup.tx_prefix[0];
    if (reply[0] == 0x7fU) {
      if ((length >= 2U) && (reply[1] == sid)) {
        if ((length >= 3U) && (reply[2] == 0x78U)) {
          tss3_set_timer(now, TSS3_PENDING_TIMEOUT_US);
        } else {
          tss3_finish();
        }
      }
    } else if (reply[0] == (uint8_t)(sid + 0x40U)) {
      tss3_handle_positive_reply(reply, length, now);
    } else {
      // Ignore replies for another service.
    }
  }
}

static void tss3_process_frame(const uint8_t *frame, uint32_t now) {
  uint8_t frame_type = frame[0] >> 4;
  if ((frame_type == 3U) && (tss3_bringup.tx_state == TSS3_TX_WAIT_FC)) {
    uint8_t flow_status = frame[0] & 0x0fU;
    if (flow_status == 0U) {
      tss3_bringup.tx_state = TSS3_TX_SENDING;
      tss3_bringup.tx_block_remaining = frame[1];
      if (frame[2] <= 0x7fU) {
        tss3_bringup.tx_stmin_us = (uint32_t)frame[2] * 1000U;
      } else if ((frame[2] >= 0xf1U) && (frame[2] <= 0xf9U)) {
        tss3_bringup.tx_stmin_us = (uint32_t)(frame[2] - 0xf0U) * 100U;
      } else {
        tss3_bringup.tx_stmin_us = 0U;
      }
      tss3_bringup.tx_next_us = now;
    } else if (flow_status == 1U) {
      tss3_set_timer(now, TSS3_RESPONSE_TIMEOUT_US);
    } else {
      tss3_finish();
    }
  } else if (frame_type == 0U) {
    uint8_t length = frame[0] & 0x0fU;
    if ((length > 0U) && (length <= 7U)) {
      tss3_handle_response(&frame[1], length, now);
    }
  } else if (frame_type == 1U) {
    uint16_t expected = ((uint16_t)(frame[0] & 0x0fU) << 8) | frame[1];
    if ((expected > 6U) && (expected <= sizeof(tss3_bringup.rx_data))) {
      tss3_bringup.rx_expected = expected;
      tss3_bringup.rx_length = 6U;
      tss3_bringup.rx_sequence = 1U;
      (void)memcpy(tss3_bringup.rx_data, &frame[2], 6U);
      const uint8_t flow_control[8] = {[0] = 0x30U};
      tss3_send_can(flow_control);
      tss3_set_timer(now, TSS3_RESPONSE_TIMEOUT_US);
    }
  } else if ((frame_type == 2U) && (tss3_bringup.rx_expected != 0U)) {
    if ((frame[0] & 0x0fU) != tss3_bringup.rx_sequence) {
      tss3_finish();
    } else {
      tss3_bringup.rx_sequence = (tss3_bringup.rx_sequence + 1U) & 0x0fU;
      const uint16_t count = MIN(7U, (uint16_t)(tss3_bringup.rx_expected - tss3_bringup.rx_length));
      (void)memcpy(&tss3_bringup.rx_data[tss3_bringup.rx_length], &frame[1], count);
      tss3_bringup.rx_length += count;
      if (tss3_bringup.rx_length == tss3_bringup.rx_expected) {
        tss3_bringup.rx_expected = 0U;
        tss3_handle_response(tss3_bringup.rx_data, tss3_bringup.rx_length, now);
      }
    }
  } else {
    // No receiver for this frame type.
  }
}

static void tss3_send_consecutive_frame(uint32_t now) {
  uint8_t frame[8] = {0};
  frame[0] = 0x20U | tss3_bringup.tx_sequence;
  tss3_send_tx_frame(frame, 1U);
  tss3_bringup.tx_sequence = (tss3_bringup.tx_sequence + 1U) & 0x0fU;
  const bool last_in_block = tss3_bringup.tx_block_remaining == 1U;
  if (tss3_bringup.tx_block_remaining > 0U) {
    tss3_bringup.tx_block_remaining -= 1U;
  }
  if (tss3_bringup.tx_offset == tss3_bringup.tx_length) {
    tss3_tx_complete(now);
  } else if (last_in_block) {
    tss3_bringup.tx_state = TSS3_TX_WAIT_FC;
    tss3_set_timer(now, TSS3_RESPONSE_TIMEOUT_US);
  } else {
    tss3_bringup.tx_next_us = now + tss3_bringup.tx_stmin_us;
  }
}

static void tss3_bringup_rx(const CANPacket_t *packet) {
  if ((packet->bus == TSS3_BUS) && (GET_LEN(packet) == 8U)) {
    if ((packet->addr != TSS3_TX_ADDR) && (packet->addr != TSS3_RX_ADDR) &&
        tss3_bringup_enabled() && (tss3_retained.wake == TSS3_WAKE_MAGIC) &&
        (tss3_bringup.phase == TSS3_PHASE_IDLE)) {
      tss3_bringup.start_pending = true;
    }
    if ((packet->addr == TSS3_RX_ADDR) &&
        (tss3_bringup_active() || tss3_bringup.start_pending)) {
      uint8_t next = (tss3_bringup.rx_write + 1U) % TSS3_RX_QUEUE_SIZE;
      if (next == tss3_bringup.rx_read) {
        tss3_bringup.rx_overflow = true;
      } else {
        (void)memcpy(tss3_bringup.rx_queue[tss3_bringup.rx_write].data, packet->data, 8U);
        tss3_bringup.rx_write = next;
      }
    }
  }
}

static void tss3_poll_active(uint32_t now) {
  while (tss3_bringup.rx_read != tss3_bringup.rx_write) {
    tss3_rx_frame_t frame;
    ENTER_CRITICAL();
    frame = tss3_bringup.rx_queue[tss3_bringup.rx_read];
    tss3_bringup.rx_read = (tss3_bringup.rx_read + 1U) % TSS3_RX_QUEUE_SIZE;
    EXIT_CRITICAL();
    tss3_process_frame(frame.data, now);
  }

  if ((tss3_bringup.tx_state == TSS3_TX_SENDING) &&
      (get_ts_elapsed(now, tss3_bringup.tx_next_us) < 0x80000000U)) {
    tss3_send_consecutive_frame(now);
  }

  if (tss3_timer_expired(now)) {
    if (tss3_bringup.phase == TSS3_PHASE_BOOT_DELAY) {
      tss3_send_session(tss3_session_extended, TSS3_PHASE_BOOT_PROBE, now);
    } else if (tss3_bringup.phase == TSS3_PHASE_TX_DRAIN) {
      tss3_finish();
    } else {
      if ((tss3_bringup.tx_state == TSS3_TX_WAIT_FC) || (tss3_bringup.tx_state == TSS3_TX_WAIT_REPLY)) {
        const uint32_t elapsed = get_ts_elapsed(now, tss3_bringup.run_started_us);
        if (((tss3_bringup.phase == TSS3_PHASE_BOOT_PROBE) && (elapsed < TSS3_BOOT_TIMEOUT_US)) ||
            ((tss3_bringup.phase == TSS3_PHASE_APP_EXTENDED) && (elapsed < TSS3_PENDING_TIMEOUT_US))) {
          tss3_send_session(tss3_session_extended, tss3_bringup.phase, now);
        } else {
          tss3_finish();
        }
      }
    }
  }
}

static void tss3_bringup_poll(uint32_t now) {
  if (!tss3_bringup_active()) {
    if (tss3_bringup_enabled() && !harness_check_ignition() && !ignition_can) {
      tss3_retained.wake = TSS3_WAKE_MAGIC;
    }
    if (tss3_bringup.start_pending) {
      if (tss3_bringup.rx_overflow) {
        tss3_clear_pending_rx();
      } else {
        tss3_begin(now);
      }
    }
  }
  if (tss3_bringup_active()) {
    if (tss3_bringup.rx_overflow) {
      tss3_finish();
    } else {
      tss3_poll_active(now);
    }
  }
}

#endif  // BOARD_TSS3_BRINGUP_H
