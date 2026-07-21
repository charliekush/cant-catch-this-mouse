/**
 * @file Arduino_RouterBridge.h
 * @author Charlie Kushelevsky (ckushelevsky@ucsd.edu)
 * @brief implements MCU functions for motor control from MPU
 * @version 0.1
 * @date 2026-07-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <Arduino.h>
#include "motors.h"

// Pin map confirmed by polarity_test.ino against ELEGOO's reference code:
// only one direction pin per channel is exposed; channel A is physically
// RIGHT, channel B is physically LEFT. Both positive-is-forward.
static const int PWMA = 5, AIN1 = 7;   // right motor
static const int PWMB = 6, BIN1 = 8;   // left motor
static const int STBY = 3;

static const float RIGHT_MOTOR_TRIM = 1.0f;   // tune: >1 boosts weak right side

static void set_side(int pwm_pin, int dir_pin, int value) {
    bool forward = value >= 0;
    digitalWrite(dir_pin, forward ? HIGH : LOW);
    int mag = abs(value);
    if (mag > 255) mag = 255;
    analogWrite(pwm_pin, mag);
}