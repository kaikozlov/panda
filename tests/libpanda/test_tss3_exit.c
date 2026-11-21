#include <assert.h>
#include "panda.c"

static bool harness_check_ignition(void) { return true; }
void set_safety_mode(uint16_t mode, uint16_t param) {
  assert(set_safety_hooks(mode, param) == 0);
}

#ifdef __APPLE__
#define section(name) section("__DATA,__backup_sram")
#endif
#include "board/tss3_bringup.h"

static void host_selects_diagnostics(void) {
  set_safety_mode(SAFETY_ELM327, 0U);
  heartbeat_disabled = true;
}

static void check_exit(tss3_phase_t initial, bool overflow) {
  tss3_bringup = (tss3_bringup_t){.phase = initial, .rx_overflow = overflow};
  host_selects_diagnostics();
  tss3_bringup_poll(0U);
  assert(tss3_bringup.phase == TSS3_PHASE_IDLE);
  assert(safety_setup.current_safety_mode == SAFETY_SILENT);
  assert(!heartbeat_disabled);

  host_selects_diagnostics();
  tss3_bringup_poll(1U);
  assert(safety_setup.current_safety_mode == SAFETY_ELM327);
  assert(heartbeat_disabled);
}

int main(void) {
  check_exit(TSS3_PHASE_TX_DRAIN, false);
  check_exit(TSS3_PHASE_APP_EXTENDED, true);

  tss3_bringup = (tss3_bringup_t){.start_pending = true, .rx_overflow = true};
  tss3_retained.wake = TSS3_WAKE_MAGIC;
  host_selects_diagnostics();
  tss3_bringup_poll(0U);
  assert(tss3_bringup.phase == TSS3_PHASE_IDLE);
  assert(!tss3_bringup.start_pending);
  assert(tss3_retained.wake == 0U);
  assert(safety_setup.current_safety_mode == SAFETY_ELM327);
  assert(heartbeat_disabled);
  return 0;
}
