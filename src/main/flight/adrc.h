/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"
#include "common/time.h"
#include "flight/pid.h"

// Per-axis runtime state (ESO + TD)
typedef struct adrcAxisState_s {
    float v_hat;        // ESO velocity estimate (deg/s)
    float sigma_hat;    // ESO disturbance estimate (mixer units)
    float u_act_hat;    // previous control output (mixer units)
    float v_ref;        // TD-filtered setpoint (deg/s)
} adrcAxisState_t;

// Computed coefficients — shared and per-axis
typedef struct adrcRuntime_s {
    float dT;
    float l0;                           // ESO correction gain: 4*pi*f_ESO (shared)
    float l1;                           // ESO base integral gain: 4*pi^2*f_ESO^2 (shared, divided by alpha per cycle)
    float td_gain;                      // TD pole: 2*pi*f_TD, 0 = disabled (shared)
    float hover_throttle;               // normalised hover throttle [0,1] — alpha reference point
    float sigma_decay;                  // leaky integrator rate (1/s); bleeds sigma_hat to 0 when undisturbed
    float itermLimit;
    float itermLimitYaw;

    // Per-axis coefficients
    float alpha_hat[XYZ_AXIS_COUNT];
    float kt[XYZ_AXIS_COUNT];
    float kd[XYZ_AXIS_COUNT];   // rate-damping gain (DTERM_SCALE * adrc_kd), opposes gyroRate directly

    // Setpoint array — written by pidController() each cycle with level-mode adjustments applied
    float setpoint[XYZ_AXIS_COUNT];

    adrcAxisState_t axis[XYZ_AXIS_COUNT];
} adrcRuntime_t;

extern adrcRuntime_t adrcRuntime;

void adrcInitProfile(const pidProfile_t *pidProfile);
void adrcController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);
void adrcResetState(void);
