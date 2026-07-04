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

// Per-axis runtime state (3-state ESO + TD)
typedef struct adrcAxisState_s {
    float z1;           // ESO rate estimate (deg/s)
    float z2;           // ESO angular-acceleration estimate (deg/s^2)
    float z3;           // ESO lumped-disturbance estimate (deg/s^3, i.e. rate_ddot units)
    float u_act_hat;    // previous control output (mixer units) — fed into the ESO's b0*u term
    float v_ref;        // TD-filtered setpoint (deg/s)
    float err_lp;       // low-pass filtered ESO error (deg/s) — feeds sigma_decay scheduling
} adrcAxisState_t;

// Computed coefficients — shared and per-axis
typedef struct adrcRuntime_s {
    float dT;
    // 3-state (degree-2) ESO gains; triple pole at -wo, wo = 2*pi*f_ESO (shared)
    float beta1;                        // 3*wo
    float beta2;                        // 3*wo^2
    float beta3;                        // wo^3
    float td_gain;                      // TD pole: 2*pi*f_TD, 0 = disabled (shared)
    float hover_throttle;               // normalised hover throttle [0,1] — b0 reference point
    float sigma_decay;                  // leaky-integrator BASE rate (1/s) on z3; bleeds it to 0 once settled
    float decaySchedGain;               // 1/(deg/s); scales the decay down while err_lp stays persistently large. 0 = legacy constant decay
    float decayFiltAlpha;               // per-cycle low-pass coefficient (fixed corner freq) used to compute err_lp
    float itermLimit;
    float itermLimitYaw;

    // Liftoff gate: while grounded the airframe can't rotate the way the model
    // assumes, so the ESO's "control actually applied" (b0*u) term is withheld
    // until liftoff is detected (see adrcController). Re-arms after a sustained
    // return to idle throttle, independent of adrcResetState().
    bool liftoff;
    float gyroActiveS;                  // seconds the gyro has been continuously above the liftoff threshold
    float idleS;                        // seconds throttle has been continuously at/below idle — re-arms the gate

    // Per-axis coefficients
    float b0_hat[XYZ_AXIS_COUNT];       // plant gain at hover: rate_ddot per output unit (throttle-scaled per cycle)
    float kp[XYZ_AXIS_COUNT];           // controller stiffness: wc^2, wc = 2*pi*f_ctrl
    float kd[XYZ_AXIS_COUNT];           // controller damping: 2*wc

    // Setpoint array — written by pidController() each cycle with level-mode adjustments applied
    float setpoint[XYZ_AXIS_COUNT];

    adrcAxisState_t axis[XYZ_AXIS_COUNT];
} adrcRuntime_t;

extern adrcRuntime_t adrcRuntime;

void adrcInitProfile(const pidProfile_t *pidProfile);
void adrcController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);
void adrcResetState(void);
