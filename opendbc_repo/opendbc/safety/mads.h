#pragma once

#include "opendbc/safety/declarations.h"

// MADS: lateral-only engagement, enabled by ALT_EXP_ENABLE_MADS.
// Only supported on the Toyota safety mode: the ACC main parse that gates lateral is Toyota only.
// Invariant: lateral permission implies ACC main is on. It is granted on a rising edge of ACC main or of
// controls_allowed (only while ACC main is on), and revoked whenever ACC main is off, on rx check failures,
// on safety mode changes, and when openpilot stops reporting MADS in its heartbeat. Brake and gas never affect it.
// Toyota SecOC does not parse ACC main (acc_main_on stays false), so it never grants lateral.
// Longitudinal control keeps using controls_allowed only.
// The state is in SafetyState, so the dual computation covers it: it runs on the current copy, like every hook.

// called once per rx message, after the safety mode rx hook updated acc_main_on and controls_allowed
static void mads_rx_update(void) {
  const bool mads_enabled = ((((uint32_t)safety_setup.alternative_experience) & ALT_EXP_ENABLE_MADS) != 0U) &&
                            (safety_setup.current_safety_mode == SAFETY_TOYOTA);
  const bool acc_main_rising = safety.acc_main_on && !safety.mads_acc_main_prev;
  const bool controls_allowed_rising = safety.controls_allowed && !safety.mads_controls_allowed_prev;

  if (!mads_enabled || !safety.acc_main_on) {
    safety.controls_allowed_lateral = false;
  } else if (acc_main_rising || controls_allowed_rising) {
    safety.controls_allowed_lateral = true;
    safety.heartbeat_engaged_mads_mismatches = 0U;
  } else {
    // no edge: keep the current state
  }

  safety.mads_acc_main_prev = safety.acc_main_on;
  safety.mads_controls_allowed_prev = safety.controls_allowed;
}

// 1Hz, from heartbeat_check_copy: exit lateral control if openpilot has not reported MADS as enabled for a few seconds
static void mads_heartbeat_check_copy(void) {
  if (safety.controls_allowed_lateral && !safety.heartbeat_engaged_mads) {
    safety.heartbeat_engaged_mads_mismatches += 1U;
    if (safety.heartbeat_engaged_mads_mismatches >= 3U) {
      safety.controls_allowed_lateral = false;
    }
  } else {
    safety.heartbeat_engaged_mads_mismatches = 0U;
  }
}
