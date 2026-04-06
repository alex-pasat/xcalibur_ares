/**
 * @file test.h
 * @brief Hardware-in-the-loop test functions for motor and stepper control.
 *
 * Call these from main.c inside a #ifdef TEST_BUILD block.
 * Each function runs its tests and prints PASS/FAIL over UART.
 *
 * Individual tests:
 *   Test_Encoders()        — all four DC motor encoders
 *   Test_CurrentSensing()  — pitch and clamp ADC current sense
 *   Test_PIDs()            — PID speed tracking for all DC motors
 *   Test_Stepper()         — underpass stepper position and stop
 *   Test_LimitSwitch()     — underpass limit switch debounce
 *   Test_HallSensors()     — pitch, roll, yaw hall effect sensors
 *   Test_AdcDma()          — ADC DMA buffer validity and live updates
 *
 * Or run everything at once:
 *   Test_RunAll()          — calls all of the above, prints summary
 */

#ifndef TEST_H
#define TEST_H

#include "robot_control.h"
#include "test.h"
void Test_Encoders(void);
void Test_CurrentSensing(void);
void Test_Characterization(void);
void Test_Stepper(void);
void Test_LimitSwitch(void);
void Test_AdcDma(void);

void Test_MinDuty(motor_ctrl_t *m);
void Test_EncoderCPRHall(motor_ctrl_t *m, const char *nm);
void Test_MaxSpeed(motor_ctrl_t *m, const char *nm);
void Test_MinSpeed(motor_ctrl_t *m, const char *nm);
void Test_Stiction(motor_ctrl_t *m, const char *nm);
void Test_PID(motor_ctrl_t *m, const char *nm, float setpoint);

void The_Test(void);

#endif /* TEST_H */