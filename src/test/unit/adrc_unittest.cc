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
 * Six test cases, in order of complexity:
 *
 *  1. testAdrcCoefficientsInit     – adrcInitProfile() computes l0/l1/kt/td_gain correctly
 *  2. testAdrcResetState           – adrcResetState() zeroes all per-axis state
 *  3. testAdrcZeroInputZeroOutput  – zero setpoint + zero gyro → output stays 0
 *  4. testAdrcSetpointResponse     – positive setpoint, zero gyro → positive output (P-like)
 *  5. testAdrcDisturbanceRejection – constant gyro error → sigma_hat grows, output fights error
 *  6. testAdrcSigmaHatClamping     – sigma_hat never exceeds itermLimit
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
static const float TEST_ESO_FREQ   = 10.0f;  // Hz — produces tidy numbers
static const float TEST_TD_FREQ    =  5.0f;  // Hz
static const uint8_t TEST_ADRC_KT  = 40;
static const uint8_t TEST_ITERMWINDUP = 80;

static const float EXPECTED_L0      = 4.0f * M_PIf * TEST_ESO_FREQ;
static const float EXPECTED_L1      = 4.0f * M_PIf * M_PIf * TEST_ESO_FREQ * TEST_ESO_FREQ;
static const float EXPECTED_KT      = PTERM_SCALE * (float)TEST_ADRC_KT;
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
    pidProfile->adrc_kt         = TEST_ADRC_KT;

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
    }

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
// Test 1 – adrcInitProfile() computes correct l0 / l1 / kt / td_gain
// ===========================================================================
TEST(adrcTest, testAdrcCoefficientsInit)
{
    resetAdrcTest();

    EXPECT_NEAR(EXPECTED_L0,       adrcRuntime.l0,           tol(EXPECTED_L0));
    EXPECT_NEAR(EXPECTED_L1,       adrcRuntime.l1,           tol(EXPECTED_L1));
    EXPECT_NEAR(EXPECTED_KT,       adrcRuntime.kt,           tol(EXPECTED_KT));
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
        adrcRuntime.axis[axis].v_hat     = 99.0f;
        adrcRuntime.axis[axis].sigma_hat = -42.0f;
        adrcRuntime.axis[axis].u_act_hat = 7.0f;
        adrcRuntime.axis[axis].v_ref     = 123.0f;
    }

    adrcResetState();

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].v_hat);
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].sigma_hat);
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
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].sigma_hat);
        EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[axis].v_hat);
    }
}

// ===========================================================================
// Test 4 – positive setpoint, zero gyro → positive output (tracking term)
//
// TD is disabled (adrc_td_freq=0) so the response is purely:
//   output = kt * (v_sp - gyroRate) = PTERM_SCALE * adrc_kt * v_sp
// This is the ADRC equivalent of a P-only step response.
// ===========================================================================
TEST(adrcTest, testAdrcSetpointResponse)
{
    resetAdrcTest();

    // Disable TD so expected output is exact: output = kt * v_sp
    pidProfile->adrc_td_freq = 0;
    pidInit(pidProfile);    // re-init to apply td_gain = 0
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);

    simulatedSetpointRate[FD_ROLL]  = 100.0f;
    simulatedSetpointRate[FD_PITCH] = 100.0f;
    simulatedSetpointRate[FD_YAW]   = 100.0f;
    // gyro stays at 0

    adrcController(pidProfile, currentTestTime());

    // With TD disabled: v_ref = v_sp instantly, v_ref_dot = 0, sigma_hat = 0
    // output = 0 + kt * (100 - 0) - 0 = kt * 100
    const float expectedOutput = PTERM_SCALE * (float)TEST_ADRC_KT * 100.0f;

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        EXPECT_NEAR(expectedOutput, pidData[axis].Sum, tol(expectedOutput));
        EXPECT_GT(pidData[axis].Sum, 0.0f);   // must be positive
    }
}

// ===========================================================================
// Test 5 – constant gyro error (simulated disturbance) → correct ESO response
//
// Setup: v_sp=0, gyroRate=10 deg/s (quad rotating despite zero setpoint).
// The ESO must:
//   a) Grow sigma_hat positive (estimating a positive disturbance)
//   b) Drive output negative (commanding counter-rotation)
//
// Uses low f_ESO=2 so sigma_hat builds up over a few observable cycles before
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

    // Run enough cycles for sigma_hat to grow meaningfully
    for (int cycle = 0; cycle < 10; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }

    // sigma_hat must be positive: ESO detects positive rate error
    EXPECT_GT(adrcRuntime.axis[FD_ROLL].sigma_hat, 0.0f);

    // I slot in pidData = -sigma_hat (disturbance cancellation term)
    EXPECT_LT(pidData[FD_ROLL].I, 0.0f);

    // Total output must be negative (fighting the positive disturbance)
    EXPECT_LT(pidData[FD_ROLL].Sum, 0.0f);

    // ESO should also be tracking the gyro rate (v_hat converges toward gyroRate)
    EXPECT_GT(adrcRuntime.axis[FD_ROLL].v_hat, 0.0f);

    // Unaffected axes stay near zero
    EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[FD_PITCH].sigma_hat);
    EXPECT_FLOAT_EQ(0.0f, adrcRuntime.axis[FD_YAW].sigma_hat);
}

// ===========================================================================
// Test 6 – sigma_hat never exceeds itermLimit regardless of error magnitude
// ===========================================================================
TEST(adrcTest, testAdrcSigmaHatClamping)
{
    resetAdrcTest();
    pidProfile->adrc_td_freq = 0;
    pidInit(pidProfile);
    adrcResetState();

    pidStabilisationState(PID_STABILISATION_ON);
    ENABLE_ARMING_FLAG(ARMED);

    // Very large gyro error to force rapid sigma_hat growth
    gyro.gyroADCf[FD_ROLL]  = 1000.0f;
    gyro.gyroADCf[FD_PITCH] = 1000.0f;
    gyro.gyroADCf[FD_YAW]   = 1000.0f;

    for (int cycle = 0; cycle < 200; cycle++) {
        adrcController(pidProfile, currentTestTime());
    }

    const float rollPitchLimit = adrcRuntime.itermLimit;
    const float yawLimit       = adrcRuntime.itermLimitYaw;

    for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
        EXPECT_LE(adrcRuntime.axis[axis].sigma_hat,  rollPitchLimit);
        EXPECT_GE(adrcRuntime.axis[axis].sigma_hat, -rollPitchLimit);
    }
    EXPECT_LE(adrcRuntime.axis[FD_YAW].sigma_hat,  yawLimit);
    EXPECT_GE(adrcRuntime.axis[FD_YAW].sigma_hat, -yawLimit);

    // Output must also be within pidSumLimit
    for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
        EXPECT_LE(fabsf(pidData[axis].Sum), (float)PIDSUM_LIMIT);
    }
    EXPECT_LE(fabsf(pidData[FD_YAW].Sum), (float)PIDSUM_LIMIT_YAW);
}
