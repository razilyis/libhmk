/*
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "pointing_device.h"

#if defined(POINTING_DEVICE_ENABLED)

#include "eeconfig.h"
#include "hardware/timer_api.h"
#include "hid.h"
#include "layout.h"
#include "sensors/pmw3610.h"
#include "split.h"
// Scroll mode: while the layer selected by the runtime `scroll_layer` config
// is active, pointer motion is sent as wheel/pan ticks instead of cursor
// movement. The keyboard.json `scroll_layer` value only seeds the persisted
// default (DEFAULT_POINTING_SCROLL_LAYER); the layer is runtime-configurable
// through SET_POINTING_CONFIG.

// Minimum interval between PMW3610 init attempts. A failed attempt blocks for
// roughly 260 ms, so retrying on every scan loop would stall key scanning and
// the split link; the cooldown bounds the retry rate to about once per second
// until the sensor comes up (e.g. after the power rail settles on a
// tether-powered slave half).
#ifndef POINTING_DEVICE_INIT_RETRY_MS
#define POINTING_DEVICE_INIT_RETRY_MS 1000
#endif

// Per-axis gain in percent (keyboard.json x_scale / y_scale). Applied after
// orientation, so X/Y refer to the keyboard axes.
#ifndef POINTING_DEVICE_X_SCALE_PERCENT
#define POINTING_DEVICE_X_SCALE_PERCENT 100
#endif
#ifndef POINTING_DEVICE_Y_SCALE_PERCENT
#define POINTING_DEVICE_Y_SCALE_PERCENT 100
#endif

// Q15 sine table for 0-90 degrees in 1-degree steps, used for the runtime
// rotation correction (round(sin(deg) * 32767)). Quadrant mapping covers the
// full circle, so no atan2 or floating point math is needed. A positive
// rotation_deg rotates the motion clockwise on the host screen and matches
// the legacy keyboard.json `angle` convention at 90/180/270 (swap/invert
// compositions).
static const int16_t sin_q15_lut[91] = {
    0,     572,   1144,  1715,  2286,  2856,  3425,  3993,  4560,  5126,
    5690,  6252,  6813,  7371,  7927,  8481,  9032,  9580,  10126, 10668,
    11207, 11743, 12275, 12803, 13328, 13848, 14364, 14876, 15383, 15886,
    16383, 16876, 17364, 17846, 18323, 18794, 19260, 19720, 20173, 20621,
    21062, 21497, 21925, 22347, 22762, 23170, 23571, 23964, 24351, 24730,
    25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 27788, 28087,
    28377, 28659, 28932, 29196, 29451, 29697, 29934, 30162, 30381, 30591,
    30791, 30982, 31163, 31335, 31498, 31650, 31794, 31927, 32051, 32165,
    32269, 32364, 32448, 32523, 32587, 32642, 32687, 32722, 32747, 32762,
    32767,
};

//--------------------------------------------------------------------+
// State
//--------------------------------------------------------------------+

static int16_t local_dx;
static int16_t local_dy;
static int16_t remote_dx;
static int16_t remote_dy;
static bool pmw3610_initialized;
static uint32_t last_init_attempt;
static pointing_config_t runtime_config;
static bool runtime_config_valid;
// Per-side orientation runtime for this half's sensor. Each half loads its own
// side slot from its local EEPROM; deltas are oriented at accumulation time so
// the split link always carries oriented counts and the master only applies
// scroll/snap after summing.
static pointing_side_config_t runtime_side;
// Cached Q15 rotation factors for runtime_side.rotation_deg
static int16_t rot_sin_q15;
static int16_t rot_cos_q15 = 32767;
// Sub-count rotation rounding remainders carried between task calls
static int16_t rot_rem_x;
static int16_t rot_rem_y;
// Suppressed minor-axis motion carried between task calls (axis snapping)
static int16_t snap_acc;
// Sub-count axis scaling remainders carried between task calls
static int16_t scale_rem_x;
static int16_t scale_rem_y;

//--------------------------------------------------------------------+
// Helpers
//--------------------------------------------------------------------+

// Keep the auto mouse layer off the scroll layer: if AML equals the scroll
// layer, ball motion pops the scroll layer and every cursor move is then
// swallowed as wheel/pan ticks, so the cursor can never move. Returns true
// when the value was corrected.
static bool pointing_device_sanitize_layers(pointing_config_t *cfg) {
  if (cfg->scroll_layer == POINTING_SCROLL_LAYER_OFF ||
      cfg->auto_mouse_layer != cfg->scroll_layer)
    return false;
  cfg->auto_mouse_layer = DEFAULT_POINTING_AUTO_MOUSE_LAYER;
  if (cfg->auto_mouse_layer == cfg->scroll_layer)
    // The build-time default AML itself collides with the configured scroll
    // layer; prefer keeping AML and disable scroll mode instead.
    cfg->scroll_layer = POINTING_SCROLL_LAYER_OFF;
  return true;
}

static uint16_t pointing_device_effective_cpi(uint16_t cpi) {
  if (cpi == 0)
    return DEFAULT_POINTING_CPI;
  return cpi;
}

// Forward declaration for the side helper used by the loaders above.
uint8_t pointing_device_my_side(void);
// Refresh the cached rotation factors and clear the transform carry-over
// state. Called whenever runtime_side is (re)loaded.
static void pointing_device_update_transform_state(void) {
  const uint16_t deg = runtime_side.rotation_deg % 360;
  const uint8_t quadrant = deg / 90;
  const uint8_t r = deg % 90;
  const int16_t sin_r = sin_q15_lut[r];
  const int16_t sin_90_r = sin_q15_lut[90 - r];

  switch (quadrant) {
  case 0:
    rot_sin_q15 = sin_r;
    rot_cos_q15 = sin_90_r;
    break;
  case 1:
    rot_sin_q15 = sin_90_r;
    rot_cos_q15 = -sin_r;
    break;
  case 2:
    rot_sin_q15 = -sin_r;
    rot_cos_q15 = -sin_90_r;
    break;
  default:
    rot_sin_q15 = -sin_90_r;
    rot_cos_q15 = sin_r;
    break;
  }

  rot_rem_x = 0;
  rot_rem_y = 0;
  snap_acc = 0;
  scale_rem_x = 0;
  scale_rem_y = 0;
}

// Load this half's side orientation from its local EEPROM, repairing a
// corrupted slot with the build-time defaults.
static void pointing_device_load_side_config(void) {
  const uint8_t side = pointing_device_my_side();
  pointing_side_config_t def = (pointing_side_config_t)DEFAULT_POINTING_SIDE_CONFIG;
  if (side == POINTING_SIDE_LEFT || side == POINTING_SIDE_RIGHT) {
    const uint8_t idx = (uint8_t)(side - 1);
    if (pointing_side_config_is_valid(&eeconfig->pointing_side[idx])) {
      runtime_side = eeconfig->pointing_side[idx];
    } else {
      runtime_side = def;
      EECONFIG_WRITE_N(pointing_side[idx], &def, sizeof(def));
    }
  } else {
    runtime_side = def;
  }
  pointing_device_update_transform_state();
}

// Load the persisted pointing global configuration, falling back to the
// build-time defaults and repairing the EEPROM copy when the persisted value
// fails validation. The global configuration is authoritative on the master
// half; the slave keeps runtime defaults until the master relays them.
static void pointing_device_load_config(void) {
  if (pointing_config_is_valid(&eeconfig->pointing_config)) {
    runtime_config = eeconfig->pointing_config;
  } else {
    runtime_config = (pointing_config_t)DEFAULT_POINTING_CONFIG;
    EECONFIG_WRITE(pointing_config, &runtime_config);
  }
  // A colliding AML/scroll layer pair can also arrive at the current version
  // (e.g. set at runtime before the collision check existed), so repair the
  // EEPROM copy the same way the invalid branch above does.
  if (pointing_device_sanitize_layers(&runtime_config))
    EECONFIG_WRITE(pointing_config, &runtime_config);
  runtime_config.cpi = pointing_device_effective_cpi(runtime_config.cpi);
  runtime_config_valid = true;
}

static void pointing_device_clear_deltas(void) {
  local_dx = 0;
  local_dy = 0;
  remote_dx = 0;
  remote_dy = 0;
}

static void pointing_device_apply_sensor_config(void) {
  if (!POINTING_DEVICE_ON_THIS_HALF || !pmw3610_initialized)
    return;

  pmw3610_set_enabled(runtime_config.enabled);
  if (runtime_config.enabled)
    pmw3610_set_cpi(pointing_device_effective_cpi(runtime_config.cpi));
}

static void pointing_device_apply_layout_config(void) {
#if defined(POINTING_DEVICE_AUTO_MOUSE_LAYER)
  layout_set_auto_mouse_enabled(runtime_config.auto_mouse_layer_enabled);
#endif
}

static void pointing_device_send_hid(int16_t dx, int16_t dy) {
  while (dx != 0 || dy != 0) {
    const int16_t sx16 = dx > 127 ? 127 : (dx < -128 ? -128 : dx);
    const int16_t sy16 = dy > 127 ? 127 : (dy < -128 ? -128 : dy);
    const int8_t sx = (int8_t)sx16;
    const int8_t sy = (int8_t)sy16;

    hid_mouse_move(sx, sy);
    hid_send_mouse_report();

    dx -= sx;
    dy -= sy;
  }
}

// Scale one axis by `percent`, carrying the sub-count remainder so slow
// motion is neither lost nor biased toward one direction.
static int16_t pointing_device_scale_axis(int16_t v, int16_t percent,
                                          int16_t *rem) {
  if (percent == 100)
    return v;
  const int32_t scaled = (int32_t)v * percent + *rem;
  int32_t out = scaled / 100;
  *rem = (int16_t)(scaled - out * 100);
  out = out > 32767 ? 32767 : (out < -32768 ? -32768 : out);
  return (int16_t)out;
}

// Apply this half's side physical-axis compensation (swap, invert, rotation)
// to raw sensor counts. Runs at accumulation time on the sensing half, so the
// split link always carries oriented counts and the master only applies
// scroll/snap after summing. Scroll conversion consumes the same oriented
// axes so wheel/pan direction follows the physical ball.
static void pointing_device_apply_orientation(int16_t *dx, int16_t *dy) {
  int16_t x = *dx;
  int16_t y = *dy;

  // Swap first, then invert, matching the sensor hardware composition.
  if (runtime_side.swap_axes) {
    const int16_t tmp = x;
    x = y;
    y = tmp;
  }
  if (runtime_side.invert_x)
    x = -x;
  if (runtime_side.invert_y)
    y = -y;

  // Axis rotation in Q15 fixed point. Sub-count rounding remainders carry
  // over between task calls so slow drags do not lose motion.
  if (rot_sin_q15 != 0 || rot_cos_q15 != 32767) {
    const int64_t fx =
        (int64_t)x * rot_cos_q15 - (int64_t)y * rot_sin_q15 + rot_rem_x;
    const int64_t fy =
        (int64_t)x * rot_sin_q15 + (int64_t)y * rot_cos_q15 + rot_rem_y;
    // Round to nearest (+0.5 LSB in Q15) before the shift
    int64_t ox64 = (fx + 16384) >> 15;
    int64_t oy64 = (fy + 16384) >> 15;
    rot_rem_x = (int16_t)(fx - (ox64 << 15));
    rot_rem_y = (int16_t)(fy - (oy64 << 15));
    ox64 = ox64 > 32767 ? 32767 : (ox64 < -32768 ? -32768 : ox64);
    oy64 = oy64 > 32767 ? 32767 : (oy64 < -32768 ? -32768 : oy64);
    x = (int16_t)ox64;
    y = (int16_t)oy64;
  }

  x = pointing_device_scale_axis(x, POINTING_DEVICE_X_SCALE_PERCENT,
                                 &scale_rem_x);
  y = pointing_device_scale_axis(y, POINTING_DEVICE_Y_SCALE_PERCENT,
                                 &scale_rem_y);

  *dx = x;
  *dy = y;
}

// Axis snapping (cursor mode only): suppress the minor axis while it stays
// below the threshold ratio of the dominant axis. Suppressed motion
// accumulates and is released once it crosses the threshold, so deliberate
// diagonal moves still pass and slow drifts are not lost. Motion on the
// minor axis alone (no dominant-axis component) passes through untouched.
static void pointing_device_apply_snap(int16_t *dx, int16_t *dy) {
  if (runtime_config.snap_axis == POINTING_SNAP_AXIS_OFF)
    return;

  int16_t x = *dx;
  int16_t y = *dy;
  const bool snap_x = runtime_config.snap_axis == POINTING_SNAP_AXIS_X;
  const int32_t major = snap_x ? x : y;
  int32_t minor = (snap_x ? y : x) + snap_acc;
  const uint32_t abs_major = (uint32_t)(major < 0 ? -major : major);
  const uint32_t abs_minor = (uint32_t)(minor < 0 ? -minor : minor);

  if (abs_minor * 100 <= abs_major * runtime_config.snap_threshold) {
    snap_acc = (int16_t)minor;
    minor = 0;
  } else {
    snap_acc = 0;
  }

  if (snap_x)
    y = (int16_t)minor;
  else
    x = (int16_t)minor;

  *dx = x;
  *dy = y;
}

// True while the runtime-configured scroll layer is active.
static bool pointing_device_in_scroll_mode(void) {
  return runtime_config.scroll_layer != POINTING_SCROLL_LAYER_OFF &&
         layout_get_current_layer() == runtime_config.scroll_layer;
}

// Sub-tick motion carried over between task calls.
static int16_t scroll_acc_x;
static int16_t scroll_acc_y;

// Convert accumulated motion into wheel/pan ticks. The vertical axis is
// inverted so rolling the ball up (cursor up, dy < 0) scrolls up (positive
// wheel); the horizontal axis maps directly to pan (positive = right).
// `invert_scroll` flips both axes ("natural" scrolling). Remainders below
// the divisor carry over so slow motion is not lost.
static void pointing_device_send_scroll(int16_t dx, int16_t dy) {
  scroll_acc_x += dx;
  scroll_acc_y += dy;

  const int16_t divisor = (int16_t)runtime_config.scroll_divisor;
  int16_t pan = scroll_acc_x / divisor;
  int16_t wheel = -scroll_acc_y / divisor;
  scroll_acc_x %= divisor;
  scroll_acc_y %= divisor;

  if (runtime_config.invert_scroll) {
    pan = -pan;
    wheel = -wheel;
  }

  while (pan != 0 || wheel != 0) {
    const int16_t p16 = pan > 127 ? 127 : (pan < -128 ? -128 : pan);
    const int16_t w16 = wheel > 127 ? 127 : (wheel < -128 ? -128 : wheel);

    hid_mouse_scroll((int8_t)w16, (int8_t)p16);
    hid_send_mouse_report();

    pan -= p16;
    wheel -= w16;
  }
}

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+

bool pointing_device_side_supported(uint8_t side) {
  if (side != POINTING_SIDE_LEFT && side != POINTING_SIDE_RIGHT)
    return false;
#if defined(POINTING_DEVICE_DUAL_SENSOR)
  return true;
#elif defined(SPLIT_KEYBOARD)
  // Single-sensor split build: only the wired half carries a sensor.
#if defined(POINTING_DEVICE_SIDE_LEFT)
  return side == POINTING_SIDE_LEFT;
#else
  return side == POINTING_SIDE_RIGHT;
#endif
#else
  // Non-split single sensor: the build-flag side owns the orientation slot.
#if defined(POINTING_DEVICE_SIDE_LEFT)
  return side == POINTING_SIDE_LEFT;
#else
  return side == POINTING_SIDE_RIGHT;
#endif
#endif
}

uint8_t pointing_device_my_side(void) {
#if defined(SPLIT_KEYBOARD)
  return split_is_left() ? POINTING_SIDE_LEFT : POINTING_SIDE_RIGHT;
#else
#if defined(POINTING_DEVICE_SIDE_LEFT)
  return POINTING_SIDE_LEFT;
#elif defined(POINTING_DEVICE_SIDE_RIGHT)
  return POINTING_SIDE_RIGHT;
#else
  return POINTING_SIDE_LEFT;
#endif
#endif
}

const pointing_side_config_t *pointing_device_get_side_runtime(void) {
  return &runtime_side;
}

void pointing_device_apply_side_local(const pointing_side_config_t *cfg) {
  if (cfg != NULL && pointing_side_config_is_valid(cfg)) {
    runtime_side = *cfg;
  } else {
    runtime_side = (pointing_side_config_t)DEFAULT_POINTING_SIDE_CONFIG;
  }
  pointing_device_update_transform_state();
}

void pointing_device_set_side_config(uint8_t side,
                                     const pointing_side_config_t *cfg) {
  if (cfg == NULL)
    return;
  pointing_side_config_t applied;
  if (pointing_side_config_is_valid(cfg)) {
    applied = *cfg;
  } else {
    applied = (pointing_side_config_t)DEFAULT_POINTING_SIDE_CONFIG;
  }
  // Apply locally when this half owns the side.
  if (side == pointing_device_my_side()) {
    runtime_side = applied;
    pointing_device_update_transform_state();
  }
#if defined(SPLIT_KEYBOARD)
  // The master relays every side update — the targeted remote side and its
  // own side alike — so both halves' EEPROM side tables converge. Without
  // the own-side relay the slave would keep a stale copy that becomes
  // authoritative if the USB role flips to that half. Persistence of both
  // copies happens in the caller (command handler) and the split RX path;
  // the per-side pending slot clears when the slave ACKs.
  if (split_is_master()) {
    split_send_pointing_side_config(side, applied.rotation_deg,
                                    applied.invert_x ? 1 : 0,
                                    applied.invert_y ? 1 : 0,
                                    applied.swap_axes ? 1 : 0);
  }
#endif
}

void pointing_device_init(void) {
  pointing_device_clear_deltas();
  pmw3610_initialized = false;
  // The subtraction wraps on purpose: the first task call may retry right
  // away instead of waiting out the full cooldown.
  last_init_attempt = timer_read() - POINTING_DEVICE_INIT_RETRY_MS;
#if defined(SPLIT_KEYBOARD)
  if (split_is_master()) {
    pointing_device_load_config();
  } else {
    runtime_config = (pointing_config_t)DEFAULT_POINTING_CONFIG;
    runtime_config.cpi = pointing_device_effective_cpi(runtime_config.cpi);
    runtime_config_valid = true;
  }
  // Each half owns its side orientation locally.
  pointing_device_load_side_config();
#else
  pointing_device_load_config();
  pointing_device_load_side_config();
#endif
  pointing_device_apply_layout_config();

#if defined(SPLIT_KEYBOARD)
  // Relay the persisted configuration to the slave half at boot so the slave
  // converges to the master's EEPROM table. The global fields are always
  // pushed (a sensor-less slave treats them as a runtime-only update), and
  // both side slots are pushed so a slave that missed runtime SETs while
  // offline still ends up with the full table.
  if (split_is_master()) {
    split_send_pointing_config(runtime_config.enabled,
                               runtime_config.auto_mouse_layer_enabled,
                               runtime_config.cpi,
                               runtime_config.auto_mouse_layer);
    for (uint8_t s = 0; s < POINTING_NUM_SIDES; s++) {
      const uint8_t side = (uint8_t)(s + 1u);
      const pointing_side_config_t *slot = &eeconfig->pointing_side[s];
      if (pointing_side_config_is_valid(slot)) {
        split_send_pointing_side_config(side, slot->rotation_deg,
                                        slot->invert_x ? 1 : 0,
                                        slot->invert_y ? 1 : 0,
                                        slot->swap_axes ? 1 : 0);
      }
    }
  }
#endif
}

void pointing_device_apply_local(const pointing_config_t *cfg) {
  if (cfg == NULL)
    return;

  if (pointing_config_is_valid(cfg)) {
    runtime_config = *cfg;
  } else {
    // Never feed an invalid configuration (e.g. a corrupt split relay payload)
    // to the sensor or layout; fall back to the build-time defaults.
    runtime_config = (pointing_config_t)DEFAULT_POINTING_CONFIG;
  }
  // Same defense for relayed / set-config values: never leave AML on the
  // scroll layer regardless of where the configuration came from.
  pointing_device_sanitize_layers(&runtime_config);
  runtime_config.cpi = pointing_device_effective_cpi(runtime_config.cpi);
  runtime_config_valid = true;
  pointing_device_apply_layout_config();
  pointing_device_apply_sensor_config();

  if (!runtime_config.enabled)
    pointing_device_clear_deltas();
}

void pointing_device_set_config(const pointing_config_t *cfg) {
  pointing_device_apply_local(cfg);

#if defined(SPLIT_KEYBOARD)
  // Persist happens in the command handler. Always relay the runtime sensor
  // fields: the slave-side sensor needs them on single-sensor (sensor on
  // slave) and dual-sensor builds alike, and a sensor-less slave applies
  // them as a harmless runtime-only update.
  split_send_pointing_config(runtime_config.enabled,
                             runtime_config.auto_mouse_layer_enabled,
                             runtime_config.cpi, runtime_config.auto_mouse_layer);
#endif
}

const pointing_config_t *pointing_device_get_config(void) {
  if (!runtime_config_valid)
    pointing_device_load_config();
  return &runtime_config;
}

// Re-read the persisted configuration and reapply it. Used when this half is
// promoted to master: pointing_device_init() may have run before USB
// enumeration settled the role and left the build-time defaults in place.
void pointing_device_reload_config(void) {
  pointing_device_load_config();
  pointing_device_load_side_config();
  pointing_device_apply_layout_config();
  pointing_device_apply_sensor_config();

  if (!runtime_config.enabled)
    pointing_device_clear_deltas();
}

void pointing_device_task(void) {
  if (POINTING_DEVICE_ON_THIS_HALF) {
    if (!pmw3610_initialized &&
        timer_elapsed(last_init_attempt) >= POINTING_DEVICE_INIT_RETRY_MS) {
      last_init_attempt = timer_read();
      pmw3610_initialized = pmw3610_init();
      if (pmw3610_initialized)
        pointing_device_apply_sensor_config();
    }

    if (!pmw3610_initialized)
      return;

    if (!runtime_config.enabled) {
      pointing_device_clear_deltas();
      return;
    }

    int16_t dx = 0;
    int16_t dy = 0;

    if (pmw3610_read_motion(&dx, &dy)) {
      // Orient at accumulation time with this half's side config so the
      // accumulator (and the split link) always holds oriented counts.
      // A half without a sensor never reaches here, so a zero delta stays
      // zero regardless of its side slot.
      pointing_device_apply_orientation(&dx, &dy);
      local_dx += dx;
      local_dy += dy;
    }
  }

  if (!runtime_config.enabled) {
    pointing_device_clear_deltas();
    return;
  }

#if defined(SPLIT_KEYBOARD)
  if (split_is_master()) {
    // Both local and remote accumulators already hold oriented counts (each
    // half oriented at accumulation time), so sum directly and apply only
    // scroll/snap here. Scroll-mode and AML run on the HID-owning half only
    // because the master holds the authoritative layer state.
    int16_t total_dx = local_dx + remote_dx;
    int16_t total_dy = local_dy + remote_dy;
    local_dx = 0;
    local_dy = 0;
    remote_dx = 0;
    remote_dy = 0;
    const bool scroll_mode = pointing_device_in_scroll_mode();
#if defined(POINTING_DEVICE_AUTO_MOUSE_LAYER)
    // Enable AML from the USB master's combined motion so slave-side sensor
    // movement reaches the half that owns layer state / HID. Skip it while
    // scrolling so scroll motion does not pop the auto mouse layer.
    if (!scroll_mode && (total_dx != 0 || total_dy != 0))
      layout_set_auto_mouse_layer(runtime_config.auto_mouse_layer);
#endif
    if (scroll_mode) {
      pointing_device_send_scroll(total_dx, total_dy);
    } else {
      pointing_device_apply_snap(&total_dx, &total_dy);
      pointing_device_send_hid(total_dx, total_dy);
    }
  }
#else
  int16_t total_dx = local_dx;
  int16_t total_dy = local_dy;
  local_dx = 0;
  local_dy = 0;
  // Non-split builds always own layer state locally. Samples were already
  // oriented at accumulation time.
  const bool scroll_mode = pointing_device_in_scroll_mode();
#if defined(POINTING_DEVICE_AUTO_MOUSE_LAYER)
  if (!scroll_mode && (total_dx != 0 || total_dy != 0))
    layout_set_auto_mouse_layer(runtime_config.auto_mouse_layer);
#endif
  if (scroll_mode) {
    pointing_device_send_scroll(total_dx, total_dy);
  } else {
    pointing_device_apply_snap(&total_dx, &total_dy);
    pointing_device_send_hid(total_dx, total_dy);
  }
#endif
}

void pointing_device_get_local_delta(int16_t *dx, int16_t *dy) {
  if (!runtime_config.enabled) {
    *dx = 0;
    *dy = 0;
    local_dx = 0;
    local_dy = 0;
    return;
  }

  *dx = local_dx;
  *dy = local_dy;
  local_dx = 0;
  local_dy = 0;
}

void pointing_device_restore_local_delta(int16_t dx, int16_t dy) {
  if (!runtime_config.enabled)
    return;
  local_dx += dx;
  local_dy += dy;
}

void pointing_device_add_remote_delta(int16_t dx, int16_t dy) {
  if (!runtime_config.enabled)
    return;
  remote_dx += dx;
  remote_dy += dy;
}

#endif // defined(POINTING_DEVICE_ENABLED)
