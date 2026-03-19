/**
 * @file test.c
 * @brief Hardware-in-the-loop test implementations.
 */

#include "test.h"
#include "encoder.h"
#include "usb_device.h"
#include "robot_config.h"
#include "robot_control.h"
#include "tiny_ring_buffer.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>

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

    MotorCtrl_SetTarget(m, 1.0f);
    run_ms(m, 5000);
    float fwd = m->current_angle_deg;
    snprintf(buf, sizeof(buf), "%s enc: increases fwd (%.1f deg)", nm, fwd);
    ASSERT(fwd > 5.0f, buf);

    MotorCtrl_SetTarget(m, -1.0f);
    run_ms(m, 5000);
    float rev = m->current_angle_deg;
    snprintf(buf, sizeof(buf), "%s enc: decreases rev (%.1f deg)", nm, rev);
    ASSERT(rev < fwd, buf);

    MotorCtrl_SetTarget(m, 0.0f);
    run_ms(m, 2000);
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

// Tests linearity of duty cycle vs speed — useful for feedforward tuning
static void linearity_one(motor_ctrl_t *m, const char *nm) {
    tprintf("\r\n-- Linearity: %s --\r\n", nm);

    float duties[] = { 0.25f, 0.50f, 0.75f, 1.0f };
    float speeds[4];

    for (int i = 0; i < 4; i++) {
        DRV8251_SetDuty(m->drv, duties[i]);
        uint32_t t = HAL_GetTick();
        while (HAL_GetTick() - t < 1500) { MotorCtrl_Update(m); HAL_Delay(1); }
        speeds[i] = m->current_rps;
        tprintf("  duty=%.2f → %.2f rps\r\n", duties[i], speeds[i]);
    }

    // Check monotonically increasing
    bool monotonic = true;
    for (int i = 1; i < 4; i++)
        if (speeds[i] <= speeds[i-1]) { monotonic = false; break; }
    ASSERT(monotonic, "linearity: speed increases monotonically with duty");

    MotorCtrl_Disable(m);
}

// Tests that the motor actually brakes and doesn't just coast
static void brake_one(motor_ctrl_t *m, const char *nm) {
    tprintf("\r\n-- Brake: %s --\r\n", nm);
    char buf[96];

    // Spin up
    DRV8251_SetDuty(m->drv, 0.8f);
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() - t < 1500) { MotorCtrl_Update(m); HAL_Delay(1); }
    float running = m->current_rps;

    // Brake and measure how quickly it stops
    DRV8251_Brake(m->drv);
    uint32_t brake_time = HAL_GetTick();
    while (HAL_GetTick() - brake_time < 2000) { MotorCtrl_Update(m); HAL_Delay(1); }
    float after_brake = fabsf(m->current_rps);

    snprintf(buf, sizeof(buf), "%s brake: stops within 2s (running=%.2f after=%.2f)",
             nm, running, after_brake);
    ASSERT(after_brake < 0.1f, buf);
    tprintf("  running=%.2f  after_brake=%.2f rps\r\n", running, after_brake);

    MotorCtrl_Disable(m);
}

// Checks that the motor stays still under small disturbances with PID active
static void disturbance_one(motor_ctrl_t *m, const char *nm) {
    tprintf("\r\n-- Disturbance Rejection: %s --\r\n", nm);
    char buf[96];

    MotorCtrl_SetTarget(m, 1.0f);
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() - t < 2000) { MotorCtrl_Update(m); HAL_Delay(1); }

    float before = m->current_rps;
    prompt("Briefly resist the yaw motor shaft by hand then release");

    t = HAL_GetTick();
    while (HAL_GetTick() - t < 2000) { MotorCtrl_Update(m); HAL_Delay(1); }
    float recovered = m->current_rps;

    snprintf(buf, sizeof(buf), "%s disturbance: recovers to 1.0 rps (got %.2f)", nm, recovered);
    NEAR(recovered, 1.0f, 0.3f, buf);
    tprintf("  before=%.2f  recovered=%.2f rps\r\n", before, recovered);

    MotorCtrl_Disable(m);
}

// ============================================================================
// Public test functions
// ============================================================================

void Test_Encoders(void) {
    tprintf("\r\n=== ENCODER TESTS ===\r\n");
    // enc_one(&dc_pitch, "pitch");
    // enc_one(&dc_roll,  "roll");
    enc_one(&dc_yaw,   "yaw");
    // enc_one(&clamp,    "clamp");
}

void Test_CurrentSensing(void) {
    tprintf("\r\n=== CURRENT SENSE TESTS ===\r\n");
    curr_one(&dc_pitch, "pitch", 800,  2000);
    curr_one(&clamp,    "clamp", 600,  1500);
    tprintf("  roll, yaw: no ADC configured — skipped\r\n");
}

void Test_Characterization(void) {
    tprintf("\r\n=== MOTOR CHARACTERIZATION ===\r\n");
    linearity_one(&dc_yaw,  "yaw");
    brake_one(&dc_yaw,      "yaw");
    disturbance_one(&dc_yaw, "yaw");
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
    MotorCtrl_SetTarget(&dc_pitch, 1.5f);
    run_ms(&dc_pitch, 300);
    uint16_t after = adc_dma_buf[0];
    MotorCtrl_Disable(&dc_pitch);

    ASSERT(after != before, "ADC DMA: pitch channel updates under load");
    tprintf("  pitch ADC before=%u after=%u\r\n", before, after);
}

void Test_MinDuty(motor_ctrl_t *m) {
    tprintf("\r\n=== MIN DUTY TEST ===\r\n");
    
    for (float duty = 0.0f; duty <= 1.0f; duty += 0.005f) {
        DRV8251_SetDuty(m->drv, duty);
        HAL_Delay(1000);
        
        __disable_irq();
        int32_t before = m->enc->count;
        __enable_irq();
        HAL_Delay(200);
        __disable_irq();
        int32_t after = m->enc->count;
        __enable_irq();
        
        bool moving = (after != before);
        tprintf("duty=%.4f moving=%s\r\n", duty, moving ? "YES" : "no");
        if (moving) break;
    }
    DRV8251_SetDuty(m->drv, 0.0f);
}

void Test_EncoderCPR(motor_ctrl_t *m) {
    tprintf("\r\n=== ENCODER CPR TEST ===\r\n");

    extern tiny_ring_buffer_t usb_rx_ring_buf;

    // Reset count
    __disable_irq();
    m->enc->count = 0;
    m->enc->prev_count = 0;
    __enable_irq();

    tprintf("Motor running slowly — send any USB message to stop\r\n");

    // Run motor slowly open loop
    DRV8251_SetDuty(m->drv, m->drv->MIN_DUTY_CYCLE);

    while (!tiny_ring_buffer_count(&usb_rx_ring_buf)) {
        tprintf("count=%ld\r\n", m->enc->count);
        HAL_Delay(200);
    }
    tiny_ring_buffer_clear(&usb_rx_ring_buf);

    DRV8251_SetDuty(m->drv, 0.0f);

    tprintf("\r\nFinal count: %ld\r\n", m->enc->count);
    tprintf("Expected per output rev: %ld\r\n", m->enc->counts_per_rev);
}

void Test_MaxSpeed(motor_ctrl_t *m, const char *nm) {
    tprintf("\r\n-- Max Speed: %s --\r\n", nm);
    char buf[96];

    // Forward
    DRV8251_SetDuty(m->drv, 1.0f);
    uint32_t t = HAL_GetTick();
    uint32_t last_tick = t;
    float sum_fwd = 0.0f;
    uint32_t count_fwd = 0;

    while (HAL_GetTick() - t < 5000) {
        HAL_Delay(1);
        uint32_t now = HAL_GetTick();
        float dt = (now - last_tick) * 0.001f;
        last_tick = now;

        float rps = fabsf(Encoder_ComputeVelocityRPS(m->enc, dt));
        sum_fwd += rps;
        count_fwd++;
    }
    float avg_fwd = (count_fwd > 0) ? sum_fwd / count_fwd : 0.0f;

    snprintf(buf, sizeof(buf), "%s max speed: motor moves at 100%% duty (%.2f rps)", nm, avg_fwd);
    ASSERT(avg_fwd > 0.1f, buf);

    DRV8251_SetDuty(m->drv, 0.0f);
    HAL_Delay(2000);

    // Flush accumulated ticks
    __disable_irq();
    m->enc->prev_count = m->enc->count;
    __enable_irq();

    // Reverse
    DRV8251_SetDuty(m->drv, -1.0f);
    t = HAL_GetTick();
    last_tick = t;
    float sum_rev = 0.0f;
    uint32_t count_rev = 0;

    while (HAL_GetTick() - t < 5000) {
        HAL_Delay(1);
        uint32_t now = HAL_GetTick();
        float dt = (now - last_tick) * 0.001f;
        last_tick = now;

        float rps = fabsf(Encoder_ComputeVelocityRPS(m->enc, dt));
        sum_rev += rps;
        count_rev++;
    }
    float avg_rev = (count_rev > 0) ? sum_rev / count_rev : 0.0f;

    snprintf(buf, sizeof(buf), "%s max speed: symmetric fwd/rev (fwd=%.2f rev=%.2f)",
             nm, avg_fwd, avg_rev);
    NEAR(avg_fwd, avg_rev, 0.5f, buf);

    tprintf("  fwd=%.2f  rev=%.2f rps\r\n", avg_fwd, avg_rev);
    MotorCtrl_Disable(m);
}

void Test_MinSpeed(motor_ctrl_t *m, const char *nm) {
    tprintf("\r\n-- Min Speed: %s --\r\n", nm);
    char buf[96];

    DRV8251_SetDuty(m->drv, m->drv->MIN_DUTY_CYCLE);
    uint32_t t = HAL_GetTick();
    uint32_t last_tick = t;
    float sum = 0.0f;
    uint32_t count = 0;

    while (HAL_GetTick() - t < 5000) {
        HAL_Delay(20);
        uint32_t now = HAL_GetTick();
        float dt = (now - last_tick) * 0.001f;
        last_tick = now;

        float rps = fabsf(Encoder_ComputeVelocityRPS(m->enc, dt));
        sum += rps;
        count++;
    }
    float avg = (count > 0) ? sum / count : 0.0f;

    snprintf(buf, sizeof(buf), "%s min speed: moves at min duty (%.2f rps)", nm, avg);
    ASSERT(avg > 0.1f, buf);

    tprintf("  avg=%.2f rps\r\n", avg);
    MotorCtrl_Disable(m);
}

void Test_Stiction(motor_ctrl_t *m, const char *nm) {
    tprintf("\r\n-- Stiction: %s --\r\n", nm);

    float stiction_duty = 0.0f;
    float moving_duty   = 0.0f;

    // -----------------------------------------------------------------------
    // Phase 1: find minimum duty to START moving from rest (stiction)
    // Ramp up slowly until encoder moves
    // -----------------------------------------------------------------------
    tprintf("  Phase 1: finding stiction (start) duty...\r\n");

    for (float duty = 0.0f; duty <= 1.0f; duty += 0.01f) {
        DRV8251_SetDuty(m->drv, duty);
        HAL_Delay(1000);  // settle time at each step

        __disable_irq();
        int32_t before = m->enc->count;
        __enable_irq();

        HAL_Delay(100);

        __disable_irq();
        int32_t after = m->enc->count;
        __enable_irq();

        tprintf("  duty=%.2f count_delta=%ld\r\n", duty, after - before);

        if (abs(after - before) > 2) {  // threshold: at least 2 counts moved
            stiction_duty = duty;
            tprintf("  >> Stiction duty: %.2f\r\n", stiction_duty);
            break;
        }
    }

    DRV8251_SetDuty(m->drv, 0.0f);
    HAL_Delay(500);

    // -----------------------------------------------------------------------
    // Phase 2: find minimum duty to KEEP moving once already spinning
    // Start at stiction duty, then ramp down until it stops
    // -----------------------------------------------------------------------
    tprintf("  Phase 2: finding minimum running duty...\r\n");

    for (float duty = stiction_duty; duty >= 0.0f; duty -= 0.01f) {
        DRV8251_SetDuty(m->drv, duty);
        HAL_Delay(3000);

        __disable_irq();
        int32_t before = m->enc->count;
        __enable_irq();

        HAL_Delay(100);

        __disable_irq();
        int32_t after = m->enc->count;
        __enable_irq();

        tprintf("  duty=%.2f count_delta=%ld\r\n", duty, after - before);

        if (abs(after - before) <= 2) {  // stopped
            moving_duty = duty + 0.01f;  // last duty it was moving at
            tprintf("  >> Min running duty: %.2f\r\n", moving_duty);
            break;
        }
    }

    DRV8251_SetDuty(m->drv, 0.0f);

    // -----------------------------------------------------------------------
    // Summary
    // -----------------------------------------------------------------------
    tprintf("\r\n  === Stiction Results: %s ===\r\n", nm);
    tprintf("  Stiction (start) duty : %.2f\r\n", stiction_duty);
    tprintf("  Min running duty      : %.2f\r\n", moving_duty);
    tprintf("  Suggested KICKSTART_DUTY  = %.2f\r\n", stiction_duty + 0.05f);
    tprintf("  Suggested MIN_DUTY_CYCLE  = %.2f\r\n", moving_duty);

    char buf[96];
    snprintf(buf, sizeof(buf), "%s stiction: start duty found (%.2f)", nm, stiction_duty);
    ASSERT(stiction_duty > 0.0f && stiction_duty < 1.0f, buf);
    snprintf(buf, sizeof(buf), "%s stiction: running duty < start duty (%.2f < %.2f)",
             nm, moving_duty, stiction_duty);
    ASSERT(moving_duty < stiction_duty, buf);
}


void Test_PID(motor_ctrl_t *m, const char *nm, float tgt) {
    tprintf("\r\n-- PID: %s (%.1f rps) --\r\n", nm, tgt);
    tprintf("Gains: Kc=%.2f Ki=%.2f Kd=%.2f\r\n", m->pid.kc, m->pid.ki, m->pid.kd);
    char buf[96];

    MotorCtrl_SetTarget(m, tgt);
    run_ms(m, 5000);
    snprintf(buf, sizeof(buf), "%s PID: reaches %.1f rps (got %.2f)", nm, tgt, m->current_rps);
    NEAR(m->current_rps, tgt, 0.3f, buf);

    float half = tgt * 0.5f;
    MotorCtrl_SetTarget(m, half);
    run_ms(m, 5000);
    snprintf(buf, sizeof(buf), "%s PID: tracks step to %.1f (got %.2f)", nm, half, m->current_rps);
    NEAR(m->current_rps, half, 0.3f, buf);

    MotorCtrl_SetTarget(m, 0.0f);
    run_ms(m, 2000);
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

