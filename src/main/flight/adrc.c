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
 * Chebbi & Brière, Journal of Field Robotics 39(4), 426-456, 2022.
 *
 * System model:  v_dot = alpha(throttle) * (u + sigma)
 *
 * alpha varies with throttle because motor torque authority scales with RPM:
 *   alpha(thr) ≈ alpha_hover * (thr / hover_thr)²
 *
 * adrc_alpha_hat is tuned at hover throttle.  The controller scales it each
 * cycle using the live RC throttle so the ESO model and control law remain
 * correct across the full throttle range, eliminating the "shoots up"
 * runaway caused by alpha mismatch at low throttle.
 *
 * ESO (Eq. 19, per cycle):
 *   alpha_eff  = alpha_hat * (thr / hover_thr)²          [throttle-scaled]
 *   v_hat     += dT * (alpha_eff*(u_act+sigma_hat) + l0*(v-v_hat))
 *   sigma_hat += dT * (l1/alpha_eff) * (v-v_hat)
 *
 * Control law (Eq. 15):
 *   output = (1/alpha_eff) * (v_ref_dot + kt*(v_ref-v)) - sigma_hat
 */

#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"

#include "sensors/gyro.h"

#include "flight/pid.h"
#include "flight/adrc.h"

FAST_DATA_ZERO_INIT adrcRuntime_t adrcRuntime;

void adrcInitProfile(const pidProfile_t *pidProfile)
{
    adrcRuntime.dT = pidRuntime.dT;

    const float f_ESO = (float)pidProfile->adrc_eso_freq;
    adrcRuntime.l0 = 4.0f * M_PIf * f_ESO;
    adrcRuntime.l1 = 4.0f * M_PIf * M_PIf * f_ESO * f_ESO;

    // hover_throttle: stored as integer percent (0-100), convert to 0-1
    // Avoid division by zero — floor at 5 %
    const float hover_pct = (float)MAX(pidProfile->adrc_hover_throttle, 5);
    adrcRuntime.hover_throttle = hover_pct * 0.01f;

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcRuntime.alpha_hat[axis] = (float)pidProfile->adrc_alpha_hat[axis];
        adrcRuntime.kt[axis]        = (float)pidProfile->adrc_kt[axis];
    }

    if (pidProfile->adrc_td_freq > 0) {
        adrcRuntime.td_gain = 2.0f * M_PIf * (float)pidProfile->adrc_td_freq;
    } else {
        adrcRuntime.td_gain = 0.0f;
    }

    adrcRuntime.itermLimit    = 0.01f * pidProfile->itermWindup * pidProfile->pidSumLimit;
    adrcRuntime.itermLimitYaw = 0.01f * pidProfile->itermWindup * pidProfile->pidSumLimitYaw;
}

void adrcResetState(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        adrcRuntime.axis[axis].v_hat     = 0.0f;
        adrcRuntime.axis[axis].sigma_hat = 0.0f;
        adrcRuntime.axis[axis].u_act_hat = 0.0f;
        adrcRuntime.axis[axis].v_ref     = 0.0f;
    }
}

void FAST_CODE adrcController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(pidProfile);

    const float dT = adrcRuntime.dT;
    const float l1 = adrcRuntime.l1;

    // Throttle-dependent alpha scaling: alpha ∝ throttle²
    // rcCommand[THROTTLE] is in [1000, 2000]; normalise to [0, 1]
    const float thr = constrainf(((float)rcCommand[THROTTLE] - 1000.0f) * 0.001f, 0.0f, 1.0f);
    const float thr_ratio = thr / adrcRuntime.hover_throttle;
    // alpha_scale = (thr/hover_thr)², clamped so alpha never drops below 4% of hover value
    const float alpha_scale = constrainf(thr_ratio * thr_ratio, 0.04f, 9.0f);

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcAxisState_t *st = &adrcRuntime.axis[axis];

        // Live alpha for this axis and throttle level
        const float alpha_eff    = adrcRuntime.alpha_hat[axis] * alpha_scale;
        const float inv_alpha    = 1.0f / alpha_eff;
        const float l1_over_alpha = l1 / alpha_eff;
        const float kt           = adrcRuntime.kt[axis];

        const float gyroRate = gyro.gyroADCf[axis];
        const float v_sp     = getSetpointRate(axis);

        // 1. Tracking Differentiator (Eq. 17)
        float v_ref_dot;
        if (adrcRuntime.td_gain > 0.0f) {
            v_ref_dot  = adrcRuntime.td_gain * (v_sp - st->v_ref);
            st->v_ref += dT * v_ref_dot;
        } else {
            st->v_ref = v_sp;
            v_ref_dot = 0.0f;
        }

        // 2. ESO update (Eq. 19) with throttle-scaled alpha
        const float v_err = gyroRate - st->v_hat;
        st->v_hat     += dT * (alpha_eff * (st->u_act_hat + st->sigma_hat) + adrcRuntime.l0 * v_err);
        st->sigma_hat += dT * l1_over_alpha * v_err;

        const float sigma_limit = (axis == FD_YAW) ? adrcRuntime.itermLimitYaw : adrcRuntime.itermLimit;
        st->sigma_hat = constrainf(st->sigma_hat, -sigma_limit, sigma_limit);

        // 3. Control law (Eq. 15) with throttle-scaled 1/alpha
        const float tracking = kt * (st->v_ref - gyroRate);
        float output = inv_alpha * (v_ref_dot + tracking) - st->sigma_hat;

        const float pid_limit = (float)((axis == FD_YAW) ? pidProfile->pidSumLimitYaw : pidProfile->pidSumLimit);
        output = constrainf(output, -pid_limit, pid_limit);

        st->u_act_hat = output;

        pidData[axis].P   = inv_alpha * tracking;
        pidData[axis].I   = -st->sigma_hat;
        pidData[axis].D   = 0.0f;
        pidData[axis].F   = inv_alpha * v_ref_dot;
        pidData[axis].S   = 0.0f;
        pidData[axis].Sum = output;
    }
}
