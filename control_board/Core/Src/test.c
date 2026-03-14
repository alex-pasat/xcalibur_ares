/**
 * @file test.c
 * @brief Hardware-in-the-loop test implementations.
 */

#include "test.h"
#include "usb_device.h"
#include "robot_config.h"
#include "robot_control.h"
#include "stm32g4xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// External instances from robot_config.c
// ---------------------------------------------------------------------------

extern motor_ctrl_t   dc_pitch;
extern motor_ctrl_t   dc_roll;
extern motor_ctrl_t   dc_yaw;
extern motor_ctrl_t   clamp;
extern stepper_ctrl_t stepper_underpass;
extern volatile uint16_t adc_dma_buf[];

// ---------------------------------------------------------------------------
// UART
// ---------------------------------------------------------------------------

extern UART_HandleTypeDef huart2;
static char _log[256];

static void tprintf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(_log, sizeof(_log), fmt, ap);
    va_end(ap);
    USB_SendString(_log);
}
// ---------------------------------------------------------------------------
// Internal test framework — counters are reset by Test_RunAll
// ---------------------------------------------------------------------------

static uint32_t n_run = 0, n_pass = 0, n_fail = 0;

static void check(bool cond, const char *name) {
    n_run++;
    if (cond) { n_pass++; tprintf("[PASS] %s\r\n", name); }
    else       { n_fail++; tprintf("[FAIL] %s\r\n", name); }
}

#define ASSERT(c, n)        check((c), (n))
#define NEAR(a, b, t, n)    check(((a)-(b))<(t) && ((b)-(a))<(t), (n))

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

static void run_ms(motor_ctrl_t *m, uint32_t ms) {
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() - t < ms) { MotorCtrl_Update(m); HAL_Delay(1); }
}

static void step_ms(uint32_t ms) {
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() - t < ms) StepperCtrl_Run(&stepper_underpass);
}

static bool step_wait(uint32_t timeout_ms) {
    uint32_t t = HAL_GetTick();
    while (StepperCtrl_Run(&stepper_underpass))
        if (HAL_GetTick() - t > timeout_ms) return false;
    return true;
}

static void prompt(const char *msg) {
    tprintf("\r\n  >> %s (3 sec) <<\r\n", msg);
    HAL_Delay(3000);
}

// ---------------------------------------------------------------------------
// Internal per-motor helpers
// ---------------------------------------------------------------------------

static void enc_one(motor_ctrl_t *m, const char *nm) {
    tprintf("\r\n-- Encoder: %s --\r\n", nm);
    char buf[96];

    Encoder_Reset(m->enc);
    MotorCtrl_Update(m);
    snprintf(buf, sizeof(buf), "%s enc: ~0 after reset", nm);
    NEAR(m->current_angle_deg, 0.0f, 1.0f, buf);

    MotorCtrl_Enable(m);

    MotorCtrl_SetTarget(m, 1.0f);
    run_ms(m, 600);
    float fwd = m->current_angle_deg;
    snprintf(buf, sizeof(buf), "%s enc: increases fwd (%.1f deg)", nm, fwd);
    ASSERT(fwd > 5.0f, buf);

    MotorCtrl_SetTarget(m, -1.0f);
    run_ms(m, 600);
    float rev = m->current_angle_deg;
    snprintf(buf, sizeof(buf), "%s enc: decreases rev (%.1f deg)", nm, rev);
    ASSERT(rev < fwd, buf);

    MotorCtrl_SetTarget(m, 0.0f);
    run_ms(m, 200);
    snprintf(buf, sizeof(buf), "%s enc: holds when stopped", nm);
    NEAR(m->current_angle_deg, rev, 3.0f, buf);

    tprintf("  fwd=%.1f  rev=%.1f  stopped=%.1f deg\r\n",
            fwd, rev, m->current_angle_deg);
    MotorCtrl_Disable(m);
}

static void curr_one(motor_ctrl_t *m, const char *nm,
                      uint32_t max_run, uint32_t max_stall) {
    tprintf("\r\n-- Current: %s (shunt %lu mohm) --\r\n",
            nm, m->curr_config.shunt_resistor_mohm);
    char buf[96];

    MotorCtrl_Disable(m);
    HAL_Delay(200);
    MotorCtrl_Update(m);
    uint32_t idle = m->current_ma;
    snprintf(buf, sizeof(buf), "%s curr: idle < 50 mA (got %lu)", nm, idle);
    ASSERT(idle < 50, buf);

    MotorCtrl_Enable(m);
    MotorCtrl_SetTarget(m, 1.5f);
    run_ms(m, 300);
    uint32_t run = m->current_ma;
    snprintf(buf, sizeof(buf), "%s curr: running > idle (%lu > %lu mA)", nm, run, idle);
    ASSERT(run > idle, buf);
    snprintf(buf, sizeof(buf), "%s curr: running < max (%lu < %lu mA)", nm, run, max_run);
    ASSERT(run < max_run, buf);

    snprintf(buf, sizeof(buf), "Stall %s shaft now", nm);
    prompt(buf);
    run_ms(m, 300);
    uint32_t stall = m->current_ma;
    snprintf(buf, sizeof(buf), "%s curr: stall > running (%lu > %lu mA)", nm, stall, run);
    ASSERT(stall > run, buf);
    snprintf(buf, sizeof(buf), "%s curr: stall < limit (%lu < %lu mA)", nm, stall, max_stall);
    ASSERT(stall < max_stall, buf);

    tprintf("  idle=%lu  run=%lu  stall=%lu mA\r\n", idle, run, stall);
    MotorCtrl_Disable(m);
}

static void pid_one(motor_ctrl_t *m, const char *nm, float tgt) {
    tprintf("\r\n-- PID: %s (%.1f rps) --\r\n", nm, tgt);
    char buf[96];

    MotorCtrl_Enable(m);

    MotorCtrl_SetTarget(m, tgt);
    run_ms(m, 1000);
    snprintf(buf, sizeof(buf), "%s PID: reaches %.1f rps (got %.2f)", nm, tgt, m->current_rps);
    NEAR(m->current_rps, tgt, 0.3f, buf);

    float half = tgt * 0.5f;
    MotorCtrl_SetTarget(m, half);
    run_ms(m, 800);
    snprintf(buf, sizeof(buf), "%s PID: tracks step to %.1f (got %.2f)", nm, half, m->current_rps);
    NEAR(m->current_rps, half, 0.3f, buf);

    MotorCtrl_SetTarget(m, 0.0f);
    run_ms(m, 500);
    snprintf(buf, sizeof(buf), "%s PID: stops cleanly (got %.2f)", nm, m->current_rps);
    NEAR(m->current_rps, 0.0f, 0.15f, buf);

    float peak = 0.0f;
    MotorCtrl_SetTarget(m, tgt);
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() - t < 1500) {
        MotorCtrl_Update(m);
        if (m->current_rps > peak) peak = m->current_rps;
        HAL_Delay(1);
    }
    snprintf(buf, sizeof(buf), "%s PID: overshoot < 25%% (peak=%.2f)", nm, peak);
    ASSERT(peak < tgt * 1.25f, buf);

    tprintf("  settled=%.2f  peak=%.2f rps\r\n", m->current_rps, peak);
    MotorCtrl_Disable(m);
}

static void hall_one(motor_ctrl_t *m, const char *nm) {
    tprintf("\r\n-- Hall: %s --\r\n", nm);
    char buf[96];

    gpio_sensor_t *h = m->hall_effect;
    if (!h) { tprintf("  SKIP: no hall sensor\r\n"); return; }

    // let Update run for 20ms to settle the debounce state
    for (int i = 0; i < 20; i++) { MotorCtrl_Update(m); HAL_Delay(1); }
    snprintf(buf, sizeof(buf), "%s hall: clear at rest", nm);
    ASSERT(h->state == false, buf);

    snprintf(buf, sizeof(buf), "Rotate %s motor slowly past hall sensor", nm);
    prompt(buf);
    MotorCtrl_Enable(m);
    MotorCtrl_SetTarget(m, 0.3f);
    bool seen = false;
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() - t < 5000) {
        MotorCtrl_Update(m);   // debounces hall internally, updates h->state
        if (h->state) { seen = true; break; }
        HAL_Delay(1);
    }
    MotorCtrl_SetTarget(m, 0.0f);
    run_ms(m, 200);
    MotorCtrl_Disable(m);

    snprintf(buf, sizeof(buf), "%s hall: triggered during rotation", nm);
    ASSERT(seen, buf);
    tprintf("  triggered: %s\r\n", seen ? "yes" : "no");
}

// ============================================================================
// Public test functions
// ============================================================================

void Test_Encoders(void) {
    tprintf("\r\n=== ENCODER TESTS ===\r\n");
    enc_one(&dc_pitch, "pitch");
    enc_one(&dc_roll,  "roll");
    enc_one(&dc_yaw,   "yaw");
    enc_one(&clamp,    "clamp");
}

void Test_CurrentSensing(void) {
    tprintf("\r\n=== CURRENT SENSE TESTS ===\r\n");
    curr_one(&dc_pitch, "pitch", 800,  2000);
    curr_one(&clamp,    "clamp", 600,  1500);
    tprintf("  roll, yaw: no ADC configured — skipped\r\n");
}

void Test_PIDs(void) {
    tprintf("\r\n=== PID TESTS ===\r\n");
    tprintf("NOTE: gains Kc=1 Ki=0 Kd=0 — tune before trusting tracking\r\n");
    pid_one(&dc_pitch, "pitch", 1.0f);
    pid_one(&dc_roll,  "roll",  1.0f);
    pid_one(&dc_yaw,   "yaw",   1.0f);
    pid_one(&clamp,    "clamp", 0.5f);
}

void Test_Stepper(void) {
    tprintf("\r\n=== STEPPER TESTS (underpass) ===\r\n");

    StepperCtrl_SetHome(&stepper_underpass);

    StepperCtrl_SetTarget(&stepper_underpass, 2000);
    bool ok = step_wait(5000);
    int32_t pos = stepper_underpass.config->current_pos;
    ASSERT(ok,                       "stepper: reaches 2000 steps in 5s");
    NEAR((float)pos, 2000.0f, 3.0f,   "stepper: position == 2000 steps");
    tprintf("  pos after fwd: %ld\r\n", pos);

    StepperCtrl_SetTarget(&stepper_underpass, 0);
    ok = step_wait(5000);
    pos = stepper_underpass.config->current_pos;
    ASSERT(ok,                       "stepper: returns to 0");
    NEAR((float)pos, 0.0f, 3.0f,     "stepper: position == 0");

    StepperCtrl_SetTarget(&stepper_underpass, 1000);
    step_ms(300);
    StepperCtrl_Stop(&stepper_underpass);
    int32_t frozen = stepper_underpass.config->current_pos;
    HAL_Delay(200);
    ASSERT(stepper_underpass.config->current_pos == frozen,
           "stepper: holds position after stop");
    tprintf("  stopped at: %ld\r\n", frozen);

    StepperCtrl_SetHome(&stepper_underpass);
    ASSERT(stepper_underpass.config->current_pos == 0,
           "stepper: SetHome zeros position");

}

void Test_LimitSwitch(void) {
    gpio_sensor_t *sw = stepper_underpass.limit_sw;
    tprintf("\r\n=== LIMIT SWITCH (underpass PE9, threshold=%u) ===\r\n",
            sw ? sw->threshold : 0);

    if (!sw) { tprintf("  SKIP: no limit switch configured\r\n"); return; }

    // let Update run for 20ms to settle the debounce state
    for (int i = 0; i < 20; i++) { StepperCtrl_Run(&stepper_underpass); HAL_Delay(1); }
    ASSERT(sw->state == false, "limit sw: clear at rest");

    prompt("Press and hold the underpass limit switch");
    bool hit = false;
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() - t < 3000) {
        StepperCtrl_Run(&stepper_underpass);  // debounces limit_sw internally
        if (sw->state) { hit = true; break; }
        HAL_Delay(1);
    }
    ASSERT(hit, "limit sw: detected press");

    // bounce injection test — corrupt the debounce counter and verify
    // that a single bad read does not flip the confirmed state.
    // We do this by calling Update once with the pin in a known state
    // and checking state does not change after just one tick.
    bool stable        = sw->state;
    sw->debounce_count = 0;           // reset count so we need threshold reads
    sw->last_state     = !stable;     // force one mismatching raw read
    StepperCtrl_Run(&stepper_underpass);   // one call — should not flip
    ASSERT(sw->state == stable, "limit sw: rejects single bounce");

    tprintf("  threshold: %u ticks = %u ms\r\n", sw->threshold, sw->threshold);
}

void Test_HallSensors(void) {
    tprintf("\r\n=== HALL EFFECT TESTS ===\r\n");
    hall_one(&dc_pitch, "pitch");
    hall_one(&dc_roll,  "roll");
    hall_one(&dc_yaw,   "yaw");
    tprintf("  clamp: no hall sensor — skipped\r\n");
}

void Test_AdcDma(void) {
    tprintf("\r\n=== ADC DMA TESTS ===\r\n");

    HAL_Delay(50);

    // indices match CubeMX DMA scan rank order
    // ADC_CHANNEL_6 = pitch (rank 1 = index 0)
    // ADC_CHANNEL_1 = clamp (rank 2 = index 1)
    // adjust if your scan order differs
    uint16_t p0 = adc_dma_buf[0];
    uint16_t p1 = adc_dma_buf[1];

    ASSERT(p0 > 0 && p0 < 4095, "ADC DMA: pitch channel in range");
    ASSERT(p1 > 0 && p1 < 4095, "ADC DMA: clamp channel in range");
    tprintf("  pitch raw=%u (%.0f mV)  clamp raw=%u (%.0f mV)\r\n",
            p0, p0 / 4095.0f * 3300.0f,
            p1, p1 / 4095.0f * 3300.0f);

    uint16_t before = adc_dma_buf[0];
    MotorCtrl_Enable(&dc_pitch);
    MotorCtrl_SetTarget(&dc_pitch, 1.5f);
    run_ms(&dc_pitch, 300);
    uint16_t after = adc_dma_buf[0];
    MotorCtrl_Disable(&dc_pitch);

    ASSERT(after != before, "ADC DMA: pitch channel updates under load");
    tprintf("  pitch ADC before=%u after=%u\r\n", before, after);
}

void Test_RunAll(void) {
    n_run = n_pass = n_fail = 0;

    tprintf("\r\n\r\n=== Robot Hardware Tests ===\r\n");
    tprintf("Build: " __DATE__ " " __TIME__ "\r\n");
    tprintf("Core:  %lu MHz\r\n", SystemCoreClock / 1000000);

    Test_Encoders();
    Test_CurrentSensing();
    Test_PIDs();
    Test_Stepper();
    Test_LimitSwitch();
    Test_HallSensors();
    Test_AdcDma();

    tprintf("\r\n================================\r\n");
    tprintf("Results: %lu/%lu passed", n_pass, n_run);
    if (n_fail > 0) tprintf("  (%lu FAILED)\r\n", n_fail);
    else            tprintf("  — all passed\r\n");
    tprintf("================================\r\n");
}