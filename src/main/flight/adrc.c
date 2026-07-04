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

/*
 * Active Disturbance Rejection Control (ADRC) rate controller.
 * Linear ADRC (Gao 2003), degree-2 formulation.
 * Chebbi & Brière, Journal of Field Robotics 39(4), 426-456, 2022.
 *
 * Plant model:  rate_ddot = b0(throttle) * u + f
 *
 *   The rate loop is modelled as relative-degree-2 from the commanded output u
 *   to gyro rate. A real quad is (motor/ESC/prop lag) -> torque -> angular
 *   acceleration -> rate, i.e. rate/u ~ b0 / (s*(tau*s+1)) — a lag in series
 *   with an integrator, which IS a 2nd-order (degree-2) plant. Modelling it at
 *   degree 2 lets the observer represent that actuator lag as plant dynamics
 *   (state z2 = angular acceleration) instead of misattributing it to a phantom
 *   disturbance, which is what made an earlier relative-degree-1 (2-state)
 *   observer resonate into a limit cycle on sharp setpoint inputs.
 *
 *   f is the lumped disturbance (wind, model error, the -rate_dot/tau term of
 *   the real plant, everything not captured by b0*u). The ESO estimates it as
 *   z3 and the control law cancels it.
 *
 * b0 varies with throttle because motor authority scales with RPM:
 *   b0(thr) ~ b0_hat * (thr / hover_thr)^2
 * adrc_b0 is tuned at hover; the controller scales it each cycle from the live
 * RC throttle so the model and control gain stay calibrated across the throttle
 * range (below hover, hover b0 is used — see alpha_scale clamp).
 *
 * 3-state ESO (degree-2, per cycle). Triple pole at -wo, wo = 2*pi*f_ESO, from
 * (s+wo)^3 giving [beta1,beta2,beta3] = [3*wo, 3*wo^2, wo^3]:
 *   b0_eff = b0_hat * (thr / hover_thr)^2         [throttle-scaled]
 *   e      = z1 - gyro
 *   z1    += dT * (z2 - beta1*e)                  [z1 -> rate]
 *   z2    += dT * (z3 + b0_eff*u_fed - beta2*e)   [z2 -> angular accel]
 *   z3    += dT * (-beta3*e)                      [z3 -> disturbance]
 *
 * Control law (LADRC state feedback, closed-loop double pole at -wc,
 * wc = 2*pi*f_ctrl, so [kp,kd] = [wc^2, 2*wc]):
 *   u = (kp*(v_ref - z1) - kd*z2 - z3) / b0_eff
 *   (the -z3/b0_eff term cancels the estimated disturbance)
 *
 * Liftoff gate:
 *   While the craft is ground-constrained (sitting on the ground, held in
 *   hand), it can't rotate in response to commanded thrust the way the plant
 *   model assumes — effective torque authority is ~0. Feeding u into the ESO
 *   regardless makes z1/z2 drift from the real (near-zero) gyro, so z3 winds up
 *   to explain a phantom disturbance that isn't real — and has to unwind once
 *   genuinely airborne, producing a takeoff bounce/oscillation that is a
 *   ground-test artefact, not a flight instability. Until liftoff is detected
 *   (throttle above ADRC_LIFTOFF_THROTTLE, or sustained rotation above
 *   ADRC_LIFTOFF_GYRO_DPS for ADRC_LIFTOFF_HOLD_S), the b0*u term is withheld
 *   from the ESO. The observer still runs (z1/z2/z3 keep updating from the gyro
 *   error), it just isn't handed a bogus "control applied" term.
 *
 *   Re-arming: liftoff re-latches false after throttle stays below
 *   ADRC_LIFTOFF_IDLE_THROTTLE for ADRC_LIFTOFF_IDLE_HOLD_S, independent of
 *   adrcResetState(). This intentionally does NOT rely on Betaflight's own
 *   zeroThrottleItermReset — airmode latches wasThrottleRaised() true for the
 *   whole arm once throttle crosses the airmode activation point, which
 *   suppresses zeroThrottleItermReset (and therefore adrcResetState()) for the
 *   rest of that arm. Without its own idle timer, the gate would only ever
 *   protect the first liftoff of an arm, leaving every subsequent ground-test
 *   rep in the same arming completely unprotected. A brief throttle dip
 *   mid-flight (shorter than ADRC_LIFTOFF_IDLE_HOLD_S) does not re-arm it.
 *
 * z3 disturbance decay + scheduling (optional, adrc_sigma_decay[_sched]):
 *   z3 is a leaky integrator — it decays toward 0 at adrc_sigma_decay so a
 *   transient (a single bump) doesn't wind up and linger. A real, sustained
 *   disturbance (bent frame, prop damage, steady wind, an arm being held) needs
 *   z3 to *hold*, which the fixed decay fights. err_lp is a slow low-pass of
 *   the ESO error; it only rises under sustained error. The effective decay is:
 *     decay_eff = sigma_decay / (1 + decaySchedGain * |err_lp|)
 *   so decay_eff -> sigma_decay (fast, "settled") when err_lp is small, and
 *   -> ~0 (slow, "hold the correction") while err_lp stays large. Default
 *   adrc_sigma_decay_sched = 0 disables scheduling (decay_eff == sigma_decay).
 *
 * TUNING (dedicated adrc_* params — see README):
 *   1. adrc_b0        — plant gain, the primary knob. Output gain ~ 1/b0, so
 *                       too LOW b0 -> too much gain -> twitchy/oscillation;
 *                       too HIGH -> sluggish. Tune this first. adrc_b0_scale
 *                       is a global multiplier on top of the per-axis value
 *                       (b0 = adrc_b0[axis] * adrc_b0_scale) — a craft-level
 *                       constant for reaching gains outside adrc_b0's 1-250
 *                       range (e.g. whoops needing very small b0), day-to-day
 *                       tuning stays in adrc_b0.
 *   2. adrc_ctrl_freq — controller bandwidth (how sharp the response is), per axis.
 *   3. adrc_eso_freq  — observer bandwidth. Higher = more disturbance/lag
 *                       rejection and robustness, until gyro noise bites.
 *
 * DIAGNOSTICS: set debug_mode = ADRC to log the observer states to Blackbox —
 * roll z1/z2/z3 (debug 0-2), pitch z1/z2/z3 (3-5), yaw z3 (6), and the current
 * throttle-scaled b0 multiplier sign-tagged by the liftoff latch (7; positive
 * = airborne, negative = still gated on the ground).
 */

#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"

#include "sensors/gyro.h"

#include "build/debug.h"

#include "flight/pid.h"
#include "flight/adrc.h"

FAST_DATA_ZERO_INIT adrcRuntime_t adrcRuntime;

// Plant-gain scale: b0_hat = adrc_b0 * adrc_b0_scale. adrc_b0 is uint8, this
// scale maps the tunable range onto realistic rate_ddot-per-output-unit gains.
// adrc_b0_scale is a runtime CLI value (see pid.h); this is only the fallback
// used when it reads as 0 (corrupted/pre-upgrade config), matching the fixed
// scale this fork used before the parameter was exposed.
#define ADRC_B0_SCALE_DEFAULT 20.0f

// Corner frequency of the err_lp low-pass used for sigma_decay scheduling.
// Deliberately slow: fast enough to react within a few hundred ms to a
// disturbance that's genuinely held (e.g. wind, a bent frame), slow enough
// to reject per-cycle gyro noise and short bumps. Not exposed as a CLI
// parameter to keep the tuning surface small — see adrc_sigma_decay_sched.
#define ADRC_DECAY_FILT_HZ 10.0f

// Liftoff gate thresholds — see file header. Matches hardware-validated values
// from prior ADRC-on-Betaflight field testing (throttle ≥40%, or sustained
// rotation >20 deg/s for 25ms, either one declares liftoff).
#define ADRC_LIFTOFF_THROTTLE 0.4f
#define ADRC_LIFTOFF_GYRO_DPS 20.0f
#define ADRC_LIFTOFF_HOLD_S   0.025f

// Re-arm thresholds — see file header "Re-arming". 5 % throttle, held 200ms,
// is treated as "back on the ground / settled", independent of Betaflight's
// own (airmode-latched) zeroThrottleItermReset.
#define ADRC_LIFTOFF_IDLE_THROTTLE 0.05f
#define ADRC_LIFTOFF_IDLE_HOLD_S   0.2f

void adrcInitProfile(const pidProfile_t *pidProfile)
{
    adrcRuntime.dT = pidRuntime.dT;

    // 3-state (degree-2) ESO observer gains. Triple pole at -wo, wo = 2*pi*f_ESO,
    // from (s+wo)^3 = s^3 + 3wo s^2 + 3wo^2 s + wo^3 → [3wo, 3wo^2, wo^3].
    const float wo = 2.0f * M_PIf * (float)pidProfile->adrc_eso_freq;
    adrcRuntime.beta1 = 3.0f * wo;
    adrcRuntime.beta2 = 3.0f * wo * wo;
    adrcRuntime.beta3 = wo * wo * wo;

    // hover_throttle: stored as integer percent (0-100), convert to 0-1
    // Avoid division by zero — floor at 5 %
    const float hover_pct = (float)MAX(pidProfile->adrc_hover_throttle, 5);
    adrcRuntime.hover_throttle = hover_pct * 0.01f;

    // Global System-Gain multiplier shared by all axes (adrc_b0_scale = 0 means
    // an old/corrupted config predating this field — fall back to the legacy
    // fixed scale rather than dividing by zero downstream).
    const float b0Scale = (pidProfile->adrc_b0_scale > 0)
        ? (float)pidProfile->adrc_b0_scale
        : ADRC_B0_SCALE_DEFAULT;

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcRuntime.b0_hat[axis] = (float)pidProfile->adrc_b0[axis] * b0Scale;
        // Controller poles at -wc (double): kp = wc^2, kd = 2*wc.
        const float wc = 2.0f * M_PIf * (float)pidProfile->adrc_ctrl_freq[axis];
        adrcRuntime.kp[axis] = wc * wc;
        adrcRuntime.kd[axis] = 2.0f * wc;
    }

    if (pidProfile->adrc_td_freq > 0) {
        adrcRuntime.td_gain = 2.0f * M_PIf * (float)pidProfile->adrc_td_freq;
    } else {
        adrcRuntime.td_gain = 0.0f;
    }

    adrcRuntime.sigma_decay   = (float)pidProfile->adrc_sigma_decay * 0.1f;

    // decaySchedGain: 0 = disabled, decay_eff always equals sigma_decay (legacy behaviour)
    adrcRuntime.decaySchedGain = (float)pidProfile->adrc_sigma_decay_sched * 0.01f;
    const float filtRC = 1.0f / (2.0f * M_PIf * ADRC_DECAY_FILT_HZ);
    adrcRuntime.decayFiltAlpha = adrcRuntime.dT / (adrcRuntime.dT + filtRC);

    adrcRuntime.itermLimit    = 0.01f * pidProfile->itermWindup * pidProfile->pidSumLimit;
    adrcRuntime.itermLimitYaw = 0.01f * pidProfile->itermWindup * pidProfile->pidSumLimitYaw;
}

void adrcResetState(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        adrcRuntime.axis[axis].z1        = 0.0f;
        adrcRuntime.axis[axis].z2        = 0.0f;
        adrcRuntime.axis[axis].z3        = 0.0f;
        adrcRuntime.axis[axis].u_act_hat = 0.0f;
        adrcRuntime.axis[axis].v_ref     = 0.0f;
        adrcRuntime.axis[axis].err_lp    = 0.0f;
    }
    adrcRuntime.liftoff     = false;
    adrcRuntime.gyroActiveS = 0.0f;
    adrcRuntime.idleS       = 0.0f;
}

void FAST_CODE adrcController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    const float dT = adrcRuntime.dT;
    const float beta1 = adrcRuntime.beta1;
    const float beta2 = adrcRuntime.beta2;
    const float beta3 = adrcRuntime.beta3;

    // Throttle-dependent b0 scaling: b0 ∝ throttle²
    // rcCommand[THROTTLE] is in [1000, 2000]; normalise to [0, 1]
    const float thr = constrainf(((float)rcCommand[THROTTLE] - 1000.0f) * 0.001f, 0.0f, 1.0f);
    const float thr_ratio = thr / adrcRuntime.hover_throttle;
    // Only scale b0 UP above hover throttle — below hover, use hover b0.
    // Scaling down toward 0 at low throttle makes 1/b0 huge, producing extreme
    // output for small errors and motor overheating.
    const float b0_scale = constrainf(thr_ratio * thr_ratio, 1.0f, 9.0f);

    // Liftoff gate (see file header). Evaluated once per cycle, shared across axes.
    if (adrcRuntime.liftoff) {
        // Re-arm after a sustained return to idle throttle (see "Re-arming" in
        // the file header) so each ground-test rep within a single arm is
        // gated, not just the first.
        if (thr < ADRC_LIFTOFF_IDLE_THROTTLE) {
            adrcRuntime.idleS += dT;
            if (adrcRuntime.idleS >= ADRC_LIFTOFF_IDLE_HOLD_S) {
                adrcRuntime.liftoff     = false;
                adrcRuntime.gyroActiveS = 0.0f;
                adrcRuntime.idleS       = 0.0f;
            }
        } else {
            adrcRuntime.idleS = 0.0f;
        }
    } else {
        adrcRuntime.idleS = 0.0f;
        if (thr >= ADRC_LIFTOFF_THROTTLE) {
            adrcRuntime.liftoff = true;
        } else {
            const float gyroPeak = fmaxf(fabsf(gyro.gyroADCf[FD_ROLL]),
                fmaxf(fabsf(gyro.gyroADCf[FD_PITCH]), fabsf(gyro.gyroADCf[FD_YAW])));
            if (gyroPeak > ADRC_LIFTOFF_GYRO_DPS) {
                adrcRuntime.gyroActiveS += dT;
                if (adrcRuntime.gyroActiveS >= ADRC_LIFTOFF_HOLD_S) {
                    adrcRuntime.liftoff = true;
                }
            } else {
                adrcRuntime.gyroActiveS = 0.0f;
            }
        }
    }

    // Blackbox visibility into the observer (debug_mode = ADRC), slot [7]: the
    // current throttle-scaled b0 multiplier (b0_scale, x100), sign-tagged by
    // the liftoff latch — positive = airborne (b0*u fed to the ESO), negative
    // = still gated on the ground. Per-axis z-states are logged below.
    DEBUG_SET(DEBUG_ADRC, 7, lrintf(adrcRuntime.liftoff ? (b0_scale * 100.0f) : -(b0_scale * 100.0f)));

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcAxisState_t *st = &adrcRuntime.axis[axis];

        // Live plant gain for this axis and throttle level
        const float b0_eff  = adrcRuntime.b0_hat[axis] * b0_scale;
        const float inv_b0  = 1.0f / b0_eff;
        const float kp      = adrcRuntime.kp[axis];
        const float kd      = adrcRuntime.kd[axis];

        const float gyroRate = gyro.gyroADCf[axis];
        // Use pre-computed setpoint (angle/horizon-mode adjusted by pidController before calling us)
        const float v_sp     = adrcRuntime.setpoint[axis];

        // 1. Tracking Differentiator (optional setpoint smoothing)
        if (adrcRuntime.td_gain > 0.0f) {
            st->v_ref += dT * (adrcRuntime.td_gain * (v_sp - st->v_ref));
        } else {
            st->v_ref = v_sp;
        }

        // 2. 3-state (degree-2) ESO update with throttle-scaled b0.
        // Pre-liftoff the b0*u term is withheld (see file header) — z1/z2/z3
        // still track the gyro, they just aren't handed a bogus "control
        // applied" term while the airframe can't actually respond to it.
        const float e     = st->z1 - gyroRate;
        const float b0u   = adrcRuntime.liftoff ? (b0_eff * st->u_act_hat) : 0.0f;

        // Slow low-pass of the ESO error (gyro - z1): tracks *sustained* error
        // only. Feeds the z3 decay scheduling below.
        const float v_err = gyroRate - st->z1;
        st->err_lp += adrcRuntime.decayFiltAlpha * (v_err - st->err_lp);

        const float decay_eff = adrcRuntime.sigma_decay
            / (1.0f + adrcRuntime.decaySchedGain * fabsf(st->err_lp));

        st->z1 += dT * (st->z2 - beta1 * e);
        st->z2 += dT * (st->z3 + b0u - beta2 * e);
        st->z3 += dT * (-beta3 * e - decay_eff * st->z3);

        // Anti-windup: clamp the disturbance so its contribution to the output
        // (z3/b0_eff) can't exceed itermLimit (respects the itermWindup knob).
        const float itermLimit = (axis == FD_YAW) ? adrcRuntime.itermLimitYaw : adrcRuntime.itermLimit;
        const float z3Limit    = b0_eff * itermLimit;
        st->z3 = constrainf(st->z3, -z3Limit, z3Limit);

        // Blackbox visibility into the observer (debug_mode = ADRC): roll
        // z1/z2/z3 in [0..2], pitch z1/z2/z3 in [3..5], yaw z3 in [6].
        // z1 = estimated rate (deg/s), z2 = estimated accel, z3 = estimated
        // disturbance. Slot [7] (b0_scale, sign-tagged by liftoff) is set once
        // above the axis loop.
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_ADRC, 0, lrintf(st->z1));
            DEBUG_SET(DEBUG_ADRC, 1, lrintf(st->z2));
            DEBUG_SET(DEBUG_ADRC, 2, lrintf(st->z3));
        } else if (axis == FD_PITCH) {
            DEBUG_SET(DEBUG_ADRC, 3, lrintf(st->z1));
            DEBUG_SET(DEBUG_ADRC, 4, lrintf(st->z2));
            DEBUG_SET(DEBUG_ADRC, 5, lrintf(st->z3));
        } else { // FD_YAW
            DEBUG_SET(DEBUG_ADRC, 6, lrintf(st->z3));
        }

        // 3. Control law: u = (kp*(v_ref - z1) - kd*z2 - z3) / b0_eff
        const float pTerm = kp * (st->v_ref - st->z1);
        const float dTerm = -kd * st->z2;
        float output = (pTerm + dTerm - st->z3) * inv_b0;

        const float pid_limit = (float)((axis == FD_YAW) ? pidProfile->pidSumLimitYaw : pidProfile->pidSumLimit);
        output = constrainf(output, -pid_limit, pid_limit);

        st->u_act_hat = output;

        pidData[axis].P   = pTerm * inv_b0;
        pidData[axis].I   = -st->z3 * inv_b0;   // disturbance-cancellation term
        pidData[axis].D   = dTerm * inv_b0;     // damps the observer's accel estimate z2
        pidData[axis].F   = 0.0f;
        pidData[axis].S   = 0.0f;
        pidData[axis].Sum = output;
    }
}
