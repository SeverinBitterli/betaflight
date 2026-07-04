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
 * Unit tests for src/main/flight/adrc.c
 *
 * Twelve test cases, in order of complexity:
 *
 *  1. testAdrcCoefficientsInit         – adrcInitProfile() computes beta1/2/3, kp, kd, b0, td_gain
 *  2. testAdrcResetState               – adrcResetState() zeroes all per-axis state
 *  3. testAdrcZeroInputZeroOutput      – zero setpoint + zero gyro → output stays 0
 *  4. testAdrcSetpointResponse         – positive setpoint, zero gyro → positive output (P-like)
 *  5. testAdrcDisturbanceRejection     – constant gyro error → z3 grows, output fights error
 *  6. testAdrcSigmaHatClamping         – disturbance output term (z3/b0) never exceeds itermLimit
 *  7. testAdrcSigmaDecaySchedule       – adrc_sigma_decay_sched slows z3 decay under a sustained error
 *  8. testAdrcLiftoffGate              – pre-liftoff, the b0*u term can't drag the observer; throttle triggers liftoff
 *  9. testAdrcLiftoffGateFromRotation  – sustained rotation (no throttle) also triggers liftoff; a brief spike does not
 * 10. testAdrcLiftoffGateRearmsOnIdle  – liftoff re-latches false after a sustained return to idle throttle
 * 11. testAdrcThrottleScaling          – b0 scales with throttle², so output shrinks at higher throttle
 * 12. testAdrcAngleMode                – ANGLE_MODE → pidLevel rate setpoint → ADRC commands a leveling output
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <cmath>

#include "unittest_macros.h"
#include "gtest/gtest.h"
#include "build/debug.h"

// -- Simulated inputs injected by each test case --
float simulatedSetpointRate[3]      = { 0, 0, 0 };
float simulatedRcDeflection[3]      = { 0, 0, 0 };
float simulatedRawSetpoint[3]       = { 0, 0, 0 };
float simulatedPrevSetpointRate[3]  = { 0, 0, 0 };
float simulatedMotorMixRange        = 0.0f;
float simulatedMaxRcDeflectionAbs   = 0.0f;
float simulatedMixerGetRcThrottle   = 0.0f;
float simulatedMaxRate[3]           = { 670, 670, 670 };
bool  simulatedThrottleRaised       = true;

int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode;

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"

    #include "config/config.h"
    #include "config/config_reset.h"

    #include "drivers/sound_beeper.h"
    #include "drivers/time.h"

    #include "fc/controlrate_profile.h"
    #include "fc/core.h"
    #include "fc/rc.h"
    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/pid_init.h"
    #include "flight/adrc.h"
    #include "flight/position.h"

    #include "io/gps.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "rx/rx.h"
    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"

    // -- Globals required by pid.c / pid_init.c --
    acc_t acc;
    gyro_t gyro;
    attitudeEulerAngles_t attitude;
    rxRuntimeState_t rxRuntimeState = {};
    float rcCommand[4];

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
    PG_REGISTER(positionConfig_t, positionConfig, PG_SYSTEM_CONFIG, 4);

    bool unitLaunchControlActive = false;
    launchControlMode_e unitLaunchControlMode = LAUNCH_CONTROL_MODE_NORMAL;

    // -- Function stubs --
    float getSetpointRate(int axis)         { return simulatedSetpointRate[axis]; }
    float getMotorMixRange(void)            { return simulatedMotorMixRange; }
    bool  wasThrottleRaised(void)           { return simulatedThrottleRaised; }
    float getRcDeflection(int axis)         { return simulatedRcDeflection[axis]; }
    float getRcDeflectionRaw(int axis)      { return simulatedRcDeflection[axis]; }
    float getRcDeflectionAbs(int axis)      { return fabsf(simulatedRcDeflection[axis]); }
    float getMaxRcDeflectionAbs()           { return fabsf(simulatedMaxRcDeflectionAbs); }
    float mixerGetRcThrottle()              { return fabsf(simulatedMixerGetRcThrottle); }
    float getRawSetpoint(int axis)          { return simulatedRawSetpoint[axis]; }
    float getFeedforward(int axis)          { return simulatedSetpointRate[axis] - simulatedPrevSetpointRate[axis]; }
    float getMaxRcRate(int axis)            { UNUSED(axis); return simulatedMaxRate[axis]; }
    bool  isBelowLandingAltitude(void)      { return false; }
    bool  gyroOverflowDetected(void)        { return false; }
    bool  isLaunchControlActive(void)       { return unitLaunchControlActive; }
    void  systemBeep(bool on)               { UNUSED(on); }
    void  beeperConfirmationBeeps(uint8_t n){ UNUSED(n); }
    void  disarm(flightLogDisarmReason_e r) { UNUSED(r); }
    void  initRcProcessing(void)            { }
}

// ---------------------------------------------------------------------------

pidProfile_t *pidProfile;
int loopIter = 0;

// Computes expected ADRC coefficients from first principles so tests stay
// coupled to the formula, not to whatever constant the init happened to emit.
static const float TEST_ESO_FREQ   = 10.0f;  // Hz → observer bandwidth wo
static const float TEST_CTRL_FREQ  =  6.0f;  // Hz → controller bandwidth wc
static const float TEST_TD_FREQ    =  5.0f;  // Hz
static const uint8_t TEST_ADRC_B0  = 100;    // b0_hat = 100 * ADRC_B0_SCALE
static const uint8_t TEST_ITERMWINDUP = 80;

// Must match ADRC_B0_SCALE in adrc.c
static const float TEST_B0_SCALE = 20.0f;

// 3-state (degree-2) ESO: triple pole at -wo → [beta1,beta2,beta3]=[3wo,3wo^2,wo^3];
// controller double pole at -wc → [kp,kd]=[wc^2,2wc].
static const float TEST_WO          = 2.0f * M_PIf * TEST_ESO_FREQ;
static const float TEST_WC          = 2.0f * M_PIf * TEST_CTRL_FREQ;
static const float EXPECTED_BETA1   = 3.0f * TEST_WO;
static const float EXPECTED_BETA2   = 3.0f * TEST_WO * TEST_WO;
static const float EXPECTED_BETA3   = TEST_WO * TEST_WO * TEST_WO;
static const float EXPECTED_KP      = TEST_WC * TEST_WC;
static const float EXPECTED_KD      = 2.0f * TEST_WC;
static const float EXPECTED_B0      = (float)TEST_ADRC_B0 * TEST_B0_SCALE;
static const float EXPECTED_TD_GAIN = 2.0f * M_PIf * TEST_TD_FREQ;

static const float EXPECTED_ITERM_LIMIT     = 0.01f * TEST_ITERMWINDUP * PIDSUM_LIMIT;
static const float EXPECTED_ITERM_LIMIT_YAW = 0.01f * TEST_ITERMWINDUP * PIDSUM_LIMIT_YAW;

// 1% relative tolerance for float comparisons
static float tol(float v) { return fabsf(v * 0.01f) + 1e-6f; }

// ---------------------------------------------------------------------------

timeUs_t currentTestTime(void)
{
    return targetPidLooptime * loopIter++;
}

void setDefaultAdrcProfile(void)
{
    pgResetAll();
    pidProfile = pidProfilesMutable(1);

    // Minimum PID-level fields pidInit / pidInitFilters need
    pidProfile->pid[PID_ROLL]  = { 40, 40, 30, 65, 0 };
    pidProfile->pid[PID_PITCH] = { 58, 50, 35, 60, 0 };
    pidProfile->pid[PID_YAW]   = { 70, 45, 20, 60, 0 };
    pidProfile->pid[PID_LEVEL] = { 50, 50, 75, 50, 0 };
    pidProfile->pid[PID_YAW].I = pidProfile->pid[PID_YAW].I / 2.5f;

    pidProfile->pidSumLimit         = PIDSUM_LIMIT;
    pidProfile->pidSumLimitYaw      = PIDSUM_LIMIT_YAW;
    pidProfile->yaw_lowpass_hz      = 0;
    pidProfile->dterm_lpf1_static_hz= 100;
    pidProfile->dterm_lpf2_static_hz= 0;
    pidProfile->dterm_notch_hz      = 0;
    pidProfile->dterm_notch_cutoff  = 0;
    pidProfile->dterm_lpf1_type     = FILTER_SVF;
    pidProfile->itermWindup         = TEST_ITERMWINDUP;
    pidProfile->pidAtMinThrottle    = PID_STABILISATION_ON;
    pidProfile->angle_limit         = 60;
    pidProfile->feedforward_transition = 0;
    pidProfile->itermLimit          = 150;
    pidProfile->yawRateAccelLimit   = 0;
    pidProfile->rateAccelLimit      = 0;
    pidProfile->anti_gravity_gain   = 10;
    pidProfile->crash_time          = 500;
    pidProfile->crash_delay         = 0;
    pidProfile->crash_recovery_angle= 10;
    pidProfile->crash_recovery_rate = 100;
    pidProfile->crash_dthreshold    = 50;
    pidProfile->crash_gthreshold    = 400;
    pidProfile->crash_setpoint_threshold = 350;
    pidProfile->crash_recovery      = PID_CRASH_RECOVERY_OFF;
    pidProfile->crash_limit_yaw     = 200;
    pidProfile->horizon_limit_degrees= 135;
    pidProfile->iterm_rotation      = false;
    pidProfile->iterm_relax         = ITERM_RELAX_OFF;
    pidProfile->iterm_relax_cutoff  = 11;
    pidProfile->iterm_relax_type    = ITERM_RELAX_SETPOINT;
    pidProfile->launchControlMode   = LAUNCH_CONTROL_MODE_NORMAL;
    pidProfile->launchControlGain   = 40;
    pidProfile->level_race_mode     = false;
    pidProfile->throttle_boost      = 0;
    pidProfile->throttle_boost_cutoff = 15;

    // ADRC configuration
    pidProfile->controller_type = CONTROLLER_ADRC;
    pidProfile->adrc_eso_freq   = (uint8_t)TEST_ESO_FREQ;
    pidProfile->adrc_td_freq    = (uint8_t)TEST_TD_FREQ;
    pidProfile->adrc_ctrl_freq[FD_ROLL]  = (uint8_t)TEST_CTRL_FREQ;
    pidProfile->adrc_ctrl_freq[FD_PITCH] = (uint8_t)TEST_CTRL_FREQ;
    pidProfile->adrc_ctrl_freq[FD_YAW]   = (uint8_t)TEST_CTRL_FREQ;
    pidProfile->adrc_b0[FD_ROLL]  = TEST_ADRC_B0;
    pidProfile->adrc_b0[FD_PITCH] = TEST_ADRC_B0;
    pidProfile->adrc_b0[FD_YAW]   = TEST_ADRC_B0;
    pidProfile->adrc_hover_throttle = 45;

    gyro.targetLooptime = 8000; // 8 ms loop (125 Hz) — same as pid_unittest
}

void resetAdrcTest(void)
{
    loopIter = 0;
    pidRuntime.tpaFactor = 1.0f;
    simulatedMotorMixRange = 0.0f;

    pidStabilisationState(PID_STABILISATION_OFF);
    DISABLE_ARMING_FLAG(ARMED);

    setDefaultAdrcProfile();

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidData[axis] = {};
        simulatedSetpointRate[axis] = 0;
        simulatedRcDeflection[axis] = 0;
        simulatedRawSetpoint[axis]  = 0;
        gyro.gyroADCf[axis]         = 0;
        adrcRuntime.setpoint[axis]  = 0; // consumed directly by adrcController(); not reset by adrcResetState()
    }

    rcCommand[THROTTLE] = 1000.0f; // 0 % — matches the implicit zero-init default other tests rely on

    attitude.values.roll = 0;
    attitude.values.pitch = 0;
    attitude.values.yaw = 0;
    flightModeFlags = 0;

    // pidInit calls pidInitConfig which calls adrcInitProfile at the end
    pidInit(pidProfile);
    loadControlRateProfile();
    adrcResetState();
}

// ===========================================================================
// Test 1 – adrcInitProfile() computes correct beta1/2/3, kp, kd, b0, td_gain
// ===========================================================================
TEST(adrcTest, testAdrcCoefficientsInit)
{
    resetAdrcTest();

    EXPECT_NEAR(EXPECTED_BETA1, adrcRuntime.beta1, tol(EXPECTED_BETA1));
    EXPECT_NEAR(EXPECTED_BETA2, adrcRuntime.beta2, tol(EXPECTED_BETA2));
    EXPECT_NEAR(EXPECTED_BETA3, adrcRuntime.beta3, tol(EXPECTED_BETA3));
    // kp/kd/b0 are per-axis; all three axes set uniformly in setDefaultAdrcProfile()
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        EXPECT_NEAR(EXPECTED_KP, adrcRuntime.kp[axis],     tol(EXPECTED_KP));
        EXPECT_NEAR(EXPECTED_KD, adrcRuntime.kd[axis],     tol(EXPECTED_KD));
        EXPECT_NEAR(EXPECTED_B0, adrcRuntime.b0_hat[axis], tol(EXPECTED_B0));
    }
    EXPECT_NEAR(EXPECTED_TD_GAIN,  adrcRuntime.td_gain,      tol(EXPECTED_TD_GAIN));
    EXPECT_NEAR(EXPECTED_ITERM_LIMIT,     adrcRuntime.itermLimit,    tol(EXPECTED_ITERM_LIMIT));
    EXPECT_NEAR(EXPECTED_ITERM_LIMIT_YAW, adrcRuntime.itermLimitYaw, tol(EXPECTED_ITERM_LIMIT_YAW));
    EXPECT_NEAR(0.008f, adrcRuntime.dT, tol(0.008f));   // 8000µs → 0.008 s
}

// ===========================================================================
// Test 2 – adrcResetState() zeroes all per-axis state
// ===========================================================================
TEST(adrcTest, testAdrcResetState)
{
    resetAdrcTest();

    // Corrupt state manually
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        adrcRuntime.axis[axis].z1        = 99.0f;
        adrcRuntime.axis[axis].z2        = 12.0f;
        adrcRuntime.axis[axis].z3        = -42.0f;
        adrcRuntime.axis[axis].u_act_hat = 7.0f;
        adrcRuntime.axis[axis].v_ref     = 123.0f;
    }

    adrcResetState();

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].z1);
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].z2);
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].z3);
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].u_act_hat);
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].v_ref);
    }
}

// ===========================================================================
// Test 3 – zero setpoint + zero gyro → output stays 0 across all axes
// ===========================================================================
TEST(adrcTest, testAdrcZeroInputZeroOutput)
{
    resetAdrcTest();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);

    // All setpoints and gyro readings are already 0 from resetAdrcTest()
    for (int cycle = 0; cycle < 50; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        EXPECT_FLOAT_EQ(0.0f, pidData[axis].Sum);
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].z3);
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].z1);
    }
}

// ===========================================================================
// Test 4 – positive setpoint, zero gyro → positive output (proportional term)
//
// TD is disabled (adrc_td_freq=0) so v_ref = v_sp instantly. On the first cycle
// all observer states are 0 and gyro = 0, so the control law reduces to:
//   output = kp*(v_ref - z1) / b0_eff = kp * v_sp / b0_hat
// (b0_scale = 1 at 0 % throttle). This is the ADRC equivalent of a P-only step.
// ===========================================================================
TEST(adrcTest, testAdrcSetpointResponse)
{
    resetAdrcTest();

    // Disable TD so v_ref tracks v_sp instantly
    pidProfile->adrc_td_freq = 0;
    pidInit(pidProfile);    // re-init to apply td_gain = 0
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);

    // adrcController() consumes adrcRuntime.setpoint[] (normally written by
    // pidController before the call); set it directly since we call it here.
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        adrcRuntime.setpoint[axis] = 100.0f;
    }
    // gyro stays at 0

    adrcController(pidProfile, currentTestTime());

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        // b0_scale = 1 at 0 % throttle → b0_eff = b0_hat[axis]
        const float expectedOutput = adrcRuntime.kp[axis] * 100.0f / adrcRuntime.b0_hat[axis];
        EXPECT_NEAR(expectedOutput, pidData[axis].Sum, tol(expectedOutput));
        EXPECT_GT(pidData[axis].Sum, 0.0f);   // must be positive
    }
}

// ===========================================================================
// Test 5 – constant gyro error (simulated disturbance) → correct ESO response
//
// Setup: v_sp=0, gyroRate=10 deg/s (quad rotating despite zero setpoint).
// The ESO must:
//   a) Grow z3 positive (estimating a positive disturbance)
//   b) Drive output negative (commanding counter-rotation)
//
// Uses low f_ESO=2 so z3 builds up over a few observable cycles before
// hitting the clamp.
// ===========================================================================
TEST(adrcTest, testAdrcDisturbanceRejection)
{
    resetAdrcTest();

    // Override with low f_ESO for readable, non-instantly-clamped dynamics
    pidProfile->adrc_eso_freq = 2;
    pidProfile->adrc_td_freq  = 0;
    pidInit(pidProfile);
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);

    const float disturbance = 10.0f;
    gyro.gyroADCf[FD_ROLL] = disturbance;
    // setpoint stays at 0

    // Run enough cycles for z3 to grow meaningfully
    for (int cycle = 0; cycle < 10; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }

    // z3 must be positive: ESO detects positive rate error
    EXPECT_GT(adrcRuntime.axis[FD_ROLL].z3, 0.0f);

    // I slot in pidData = -z3 (disturbance cancellation term)
    EXPECT_LT(pidData[FD_ROLL].I, 0.0f);

    // Total output must be negative (fighting the positive disturbance)
    EXPECT_LT(pidData[FD_ROLL].Sum, 0.0f);

    // ESO should also be tracking the gyro rate (z1 converges toward gyroRate)
    EXPECT_GT(adrcRuntime.axis[FD_ROLL].z1, 0.0f);

    // Unaffected axes stay near zero
    EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[FD_PITCH].z3);
    EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[FD_YAW].z3);
}

// ===========================================================================
// Test 6 – the disturbance term's contribution to the output (z3/b0_eff, which
// is -pidData.I) never exceeds itermLimit regardless of error magnitude. z3
// itself is clamped to b0_eff*itermLimit, so it's the output contribution that
// is bounded by the itermWindup-derived limit.
// ===========================================================================
TEST(adrcTest, testAdrcSigmaHatClamping)
{
    resetAdrcTest();
    pidProfile->adrc_td_freq = 0;
    pidInit(pidProfile);
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);

    // Very large gyro error to force rapid z3 growth
    gyro.gyroADCf[FD_ROLL]  = 1000.0f;
    gyro.gyroADCf[FD_PITCH] = 1000.0f;
    gyro.gyroADCf[FD_YAW]   = 1000.0f;

    for (int cycle = 0; cycle < 200; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }

    // pidData.I = -z3/b0_eff — its magnitude must stay within itermLimit.
    // (small epsilon for float rounding at the clamp boundary)
    const float rollPitchLimit = adrcRuntime.itermLimit    + 1.0f;
    const float yawLimit       = adrcRuntime.itermLimitYaw + 1.0f;

    for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
        EXPECT_LE(fabsf(pidData[axis].I), rollPitchLimit);
    }
    EXPECT_LE(fabsf(pidData[FD_YAW].I), yawLimit);

    // Output must also be within pidSumLimit
    for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
        EXPECT_LE(fabsf(pidData[axis].Sum), (float)PIDSUM_LIMIT);
    }
    EXPECT_LE(fabsf(pidData[FD_YAW].Sum), (float)PIDSUM_LIMIT_YAW);
}

// ===========================================================================
// Test 7 – adrc_sigma_decay_sched slows the z3 leak while the ESO
// error stays persistently large (a held/sustained disturbance), so the
// correction is retained better than with the legacy fixed-rate decay.
//
// Same sustained disturbance and cycle count is run twice with a strong
// base decay (adrc_sigma_decay = 50 → 5.0 /s) so the drag is clearly
// visible: once with scheduling disabled (legacy behaviour) and once
// enabled. err_lp (the low-pass of the ESO error) stays elevated for the
// whole run because the disturbance never stops, so the scheduled run's
// effective decay is reduced throughout and z3 should end up larger.
//
// f_ESO = 1 and 20 cycles keep the 3-state observer in its initial monotonic
// rise: z3 is firmly positive for both runs. A faster observer or longer window
// lets the degree-2 transient overshoot z3 through zero, making the magnitude
// comparison meaningless.
// ===========================================================================
TEST(adrcTest, testAdrcSigmaDecaySchedule)
{
    const float disturbance = 10.0f;
    const int cycles = 20;

    // -- Run 1: scheduling disabled (legacy constant decay) --
    resetAdrcTest();
    pidProfile->adrc_eso_freq          = 1;
    pidProfile->adrc_td_freq           = 0;
    pidProfile->adrc_sigma_decay       = 50;   // strong base decay: 5.0 /s
    pidProfile->adrc_sigma_decay_sched = 0;
    pidInit(pidProfile);
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);
    gyro.gyroADCf[FD_ROLL] = disturbance;

    for (int cycle = 0; cycle < cycles; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }
    const float sigmaHatUnscheduled = adrcRuntime.axis[FD_ROLL].z3;

    // -- Run 2: scheduling enabled (decaySchedGain = 1.0 /(deg/s)) --
    resetAdrcTest();
    pidProfile->adrc_eso_freq          = 1;
    pidProfile->adrc_td_freq           = 0;
    pidProfile->adrc_sigma_decay       = 50;
    pidProfile->adrc_sigma_decay_sched = 100;
    pidInit(pidProfile);
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);
    gyro.gyroADCf[FD_ROLL] = disturbance;

    for (int cycle = 0; cycle < cycles; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }
    const float sigmaHatScheduled = adrcRuntime.axis[FD_ROLL].z3;

    // Both runs must still be fighting the positive disturbance...
    EXPECT_GT(sigmaHatUnscheduled, 0.0f);
    EXPECT_GT(sigmaHatScheduled, 0.0f);

    // ...but scheduling must retain noticeably more of the correction
    EXPECT_GT(sigmaHatScheduled, sigmaHatUnscheduled * 1.2f);
}

// ===========================================================================
// Test 8 – Liftoff gate withholds u_act_hat from z1's ESO update until
// liftoff is detected, so a stale/large "commanded output" (as if the craft
// were ground-constrained, spinning props without actually rotating) can't
// drag the observer's state. Throttle above ADRC_LIFTOFF_THROTTLE then
// latches liftoff on the very next cycle.
// ===========================================================================
TEST(adrcTest, testAdrcLiftoffGate)
{
    resetAdrcTest();
    pidProfile->adrc_td_freq = 0;
    pidInit(pidProfile);
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);

    EXPECT_FALSE(adrcRuntime.liftoff);

    // Ground-constrained scenario: 0 % throttle, craft not actually rotating,
    // but a large stale output already sitting in u_act_hat (as if props were
    // already spinning against the ground last cycle). Without the gate this
    // would drag z1 via alpha_eff * u_act_hat despite gyroRate staying ~0.
    rcCommand[THROTTLE] = 1000.0f;
    adrcRuntime.axis[FD_ROLL].u_act_hat = 400.0f;
    gyro.gyroADCf[FD_ROLL] = 0.0f;

    for (int cycle = 0; cycle < 20; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }

    EXPECT_FALSE(adrcRuntime.liftoff);
    // z1 must stay near zero — only l0*v_err can move it pre-liftoff, and
    // v_err stays ~0 here since gyroRate is 0. Without the gate, the stale
    // u_act_hat=400 alone would have moved it by ~128 in the very first cycle.
    EXPECT_NEAR(0.0f, adrcRuntime.axis[FD_ROLL].z1, 1.0f);

    // Raise throttle above the liftoff threshold — must latch on the next cycle.
    rcCommand[THROTTLE] = 1500.0f; // 50 %, above ADRC_LIFTOFF_THROTTLE (40 %)
    adrcController(pidProfile, currentTestTime());
    EXPECT_TRUE(adrcRuntime.liftoff);
}

// ===========================================================================
// Test 9 – Liftoff also triggers from sustained rotation alone (no throttle
// needed), but only once it's held for the configured hold time — a brief
// spike shorter than that must not falsely trigger it.
// ===========================================================================
TEST(adrcTest, testAdrcLiftoffGateFromRotation)
{
    resetAdrcTest();
    pidProfile->adrc_td_freq = 0;
    pidInit(pidProfile);
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);

    rcCommand[THROTTLE] = 1000.0f; // 0 % throttle throughout — never crosses the throttle gate

    // A brief rotation spike, shorter than the hold time, must not trigger liftoff.
    gyro.gyroADCf[FD_ROLL] = 100.0f; // well above the 20 deg/s threshold
    adrcController(pidProfile, currentTestTime());
    EXPECT_FALSE(adrcRuntime.liftoff);
    gyro.gyroADCf[FD_ROLL] = 0.0f;
    adrcController(pidProfile, currentTestTime());
    EXPECT_FALSE(adrcRuntime.liftoff); // interrupted — hold timer must have reset

    // Sustained rotation past the hold time (25ms) must trigger it.
    // dT = 0.008s here, so 4 cycles = 32ms > 25ms.
    gyro.gyroADCf[FD_ROLL] = 100.0f;
    for (int cycle = 0; cycle < 4; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }
    EXPECT_TRUE(adrcRuntime.liftoff);
}

// ===========================================================================
// Test 10 – Once latched, liftoff re-arms (goes back to false) after throttle
// stays below ADRC_LIFTOFF_IDLE_THROTTLE for ADRC_LIFTOFF_IDLE_HOLD_S. This is
// deliberately independent of adrcResetState()/zeroThrottleItermReset, since
// Betaflight's airmode latches wasThrottleRaised() true for the whole arm —
// without its own idle timer the gate would only ever protect the first
// liftoff of an arm, leaving repeat ground-test reps unprotected. A brief
// blip back above idle (shorter than the hold time) must reset that timer.
// ===========================================================================
TEST(adrcTest, testAdrcLiftoffGateRearmsOnIdle)
{
    resetAdrcTest();
    pidProfile->adrc_td_freq = 0;
    pidInit(pidProfile);
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);

    // Latch liftoff via throttle.
    rcCommand[THROTTLE] = 1500.0f; // 50 %, above ADRC_LIFTOFF_THROTTLE (40 %)
    adrcController(pidProfile, currentTestTime());
    EXPECT_TRUE(adrcRuntime.liftoff);

    // Drop to idle throttle for less than the hold time (0.2 s / 0.008 s =
    // 25 cycles) — must stay latched.
    rcCommand[THROTTLE] = 1010.0f; // 1 %, below ADRC_LIFTOFF_IDLE_THROTTLE (5 %)
    for (int cycle = 0; cycle < 20; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }
    EXPECT_TRUE(adrcRuntime.liftoff);

    // A blip back above the idle threshold (but below the liftoff threshold)
    // must reset the idle-hold timer.
    rcCommand[THROTTLE] = 1200.0f; // 20 %
    adrcController(pidProfile, currentTestTime());
    EXPECT_TRUE(adrcRuntime.liftoff);

    // Back to idle for the full hold time (with margin) — must re-arm.
    rcCommand[THROTTLE] = 1010.0f;
    for (int cycle = 0; cycle < 30; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }
    EXPECT_FALSE(adrcRuntime.liftoff);

    // Re-armed: a stale u_act_hat can't drag z1 again, mirroring test 8.
    adrcRuntime.axis[FD_ROLL].u_act_hat = 400.0f;
    gyro.gyroADCf[FD_ROLL] = 0.0f;
    for (int cycle = 0; cycle < 5; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }
    EXPECT_FALSE(adrcRuntime.liftoff);
    EXPECT_NEAR(0.0f, adrcRuntime.axis[FD_ROLL].z1, 1.0f);
}

// ===========================================================================
// Test 11 – throttle-scaled b0. The plant gain b0 rises with throttle² above
// hover (b0_eff = b0_hat * (thr/hover)², clamped to [1,9]), so the control gain
// (~1/b0_eff) drops as throttle rises. The same setpoint step therefore yields
// a proportionally smaller output at full throttle than at hover.
// ===========================================================================
TEST(adrcTest, testAdrcThrottleScaling)
{
    // Helper: first-cycle output for a given throttle, TD disabled.
    // At hover throttle b0_scale = 1; at full throttle it is (1/hover)² clamped.
    const float hover = 0.45f;  // matches adrc_hover_throttle = 45

    // -- Hover throttle (b0_scale = 1) --
    resetAdrcTest();
    pidProfile->adrc_td_freq = 0;
    pidInit(pidProfile);
    adrcResetState();
    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);
    rcCommand[THROTTLE] = 1000.0f + 1000.0f * hover;   // 45 %
    adrcRuntime.setpoint[FD_ROLL] = 100.0f;
    gyro.gyroADCf[FD_ROLL] = 0.0f;
    adrcController(pidProfile, currentTestTime());
    const float outputHover = pidData[FD_ROLL].Sum;

    // -- Full throttle (b0_scale = (1/hover)² clamped to <= 9) --
    resetAdrcTest();
    pidProfile->adrc_td_freq = 0;
    pidInit(pidProfile);
    adrcResetState();
    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);
    rcCommand[THROTTLE] = 2000.0f;                     // 100 %
    adrcRuntime.setpoint[FD_ROLL] = 100.0f;
    gyro.gyroADCf[FD_ROLL] = 0.0f;
    adrcController(pidProfile, currentTestTime());
    const float outputFull = pidData[FD_ROLL].Sum;

    float b0Scale = (1.0f / hover) * (1.0f / hover);
    b0Scale = constrainf(b0Scale, 1.0f, 9.0f);         // ≈ 4.94

    EXPECT_GT(outputHover, 0.0f);
    EXPECT_GT(outputFull,  0.0f);
    // Full-throttle output must be smaller by the b0 scale factor.
    EXPECT_LT(outputFull, outputHover);
    EXPECT_NEAR(outputHover / b0Scale, outputFull, tol(outputHover / b0Scale));
}

// ===========================================================================
// Test 12 – Angle mode integration. Driven through the full pidController()
// (not adrcController directly) with ANGLE_MODE active: pidLevel() must convert
// the centred stick + a real attitude error into a leveling rate setpoint, and
// the ADRC rate loop must command output in the correcting direction.
// ===========================================================================
TEST(adrcTest, testAdrcAngleMode)
{
    resetAdrcTest();
    pidProfile->adrc_td_freq = 0;   // no setpoint lag — respond immediately
    pidInit(pidProfile);
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(ANGLE_MODE);

    // Craft rolled +30° (300 decidegrees), pitch level, sticks centred, flying.
    attitude.values.roll  = 300;
    attitude.values.pitch = 0;
    simulatedSetpointRate[FD_ROLL]  = 0;   // getSetpointRate() → 0 (centred)
    simulatedSetpointRate[FD_PITCH] = 0;
    rcCommand[THROTTLE] = 1500.0f;         // mid throttle (past liftoff)

    for (int cycle = 0; cycle < 20; cycle++) {
        pidController(pidProfile, currentTestTime());
    }

    // pidLevel turned the +30° roll (centred stick) into a negative leveling
    // rate setpoint, and ADRC commands a negative output to roll back to level.
    EXPECT_LT(adrcRuntime.setpoint[FD_ROLL], 0.0f);
    EXPECT_LT(pidData[FD_ROLL].Sum, 0.0f);
    EXPECT_TRUE(pidRuntime.axisInAngleMode[FD_ROLL]);

    // Pitch is already level with a centred stick → near-zero setpoint/output.
    EXPECT_NEAR(0.0f, adrcRuntime.setpoint[FD_PITCH], 5.0f);

    DISABLE_FLIGHT_MODE(ANGLE_MODE);
}
