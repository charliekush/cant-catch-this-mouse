/**
 * @file sketch.ino
 * @brief Evasion Bot STM32 side: drives the TB6612 motor driver and exposes
 * RPC handlers to the MPU over the Bridge, with a local ultrasonic safety
 * reflex that overrides MPU motion commands independent of the Bridge link.
 * @date 2026-07-14
 */

#include "Arduino_RouterBridge.h"

// Pin map confirmed by sketch/polarity_test/polarity_test.ino against
// ELEGOO's own DeviceDriverSet_xxx0.h/.cpp reference code for this shield:
// only one direction pin per TB6612 channel is exposed to the MCU (the other
// direction input is hardwired on the shield PCB), and channel A is
// physically the RIGHT side while channel B is physically the LEFT side.
// Both channels share the same positive-is-forward convention.
const int PWMA = 5, AIN1 = 7; // right motor
const int PWMB = 6, BIN1 = 8; // left motor
const int STBY = 3;           // TB6612 standby

// Ultrasonic pins are UNVERIFIED -- confirm against the real chassis wiring
// before trusting STOP_DISTANCE_M as an actual safety margin.
const int TRIG = 12, ECHO = 13;

const float STOP_DISTANCE_M = 0.15; // hard local halt threshold

volatile bool halted = false;

/**
 * @brief Configures motor and ultrasonic pins, and registers the RPC
 * handlers the MPU calls. All three handlers touch digitalWrite/analogWrite/
 * pulseIn, so they are registered with provide_safe rather than provide, to
 * run in the main loop() context instead of the background RPC thread.
 */
void setup()
{
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    Bridge.begin();
    Bridge.provide_safe("set_motion", rpc_set_motion);
    Bridge.provide_safe("stop", rpc_stop);
    Bridge.provide_safe("get_range", rpc_get_range);
}

/**
 * @brief Fast local safety reflex, independent of the MPU: halts the motors
 * whenever the ultrasonic reading drops below STOP_DISTANCE_M, so a slow
 * vision frame on the MPU can never cause a head-on collision. Bridge RPCs
 * are serviced automatically between loop() iterations; no explicit poll
 * call is needed.
 */
void loop()
{
    float d = read_ultrasonic_m();
    if (d < STOP_DISTANCE_M)
    {
        halted = true;
        drive(0, 0);
    }
    else
    {
        halted = false;
    }
}

/**
 * @brief Drives both wheels at independent signed PWM values.
 *
 * left_pwm and right_pwm are physical sides, not TB6612 channel letters:
 * left_pwm is routed to channel B and right_pwm to channel A, per the
 * mapping confirmed in polarity_test.ino.
 *
 * @param left_pwm Signed PWM for the physically left wheel pair, in
 * [-255, 255]. Routed to channel B.
 * @param right_pwm Signed PWM for the physically right wheel pair, in
 * [-255, 255]. Routed to channel A.
 */
void drive(int left_pwm, int right_pwm)
{
    set_side(PWMB, BIN1, left_pwm);
    set_side(PWMA, AIN1, right_pwm);
}

/**
 * @brief Drives one TB6612 channel at a signed PWM value.
 *
 * @param pwm_pin PWM pin for this channel.
 * @param dir_pin Direction pin for this channel.
 * @param value Signed PWM magnitude in [-255, 255]. Positive is forward and
 * negative is reverse, confirmed for both channels by polarity_test.ino.
 */
void set_side(int pwm_pin, int dir_pin, int value)
{
    bool forward = value >= 0;
    digitalWrite(dir_pin, forward ? HIGH : LOW);
    int mag = abs(value);
    if (mag > 255)
    {
        mag = 255;
    }
    analogWrite(pwm_pin, mag);
}

/**
 * @brief Reads the front ultrasonic sensor.
 *
 * Blocks for up to 30ms waiting for the echo pulse (pulseIn timeout). Safe
 * to call from provide_safe context, which runs in the main loop().
 *
 * @return Distance to the nearest obstacle in meters, or 999.0 if no echo
 * was received within the timeout.
 */
float read_ultrasonic_m()
{
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    long us = pulseIn(ECHO, HIGH, 30000); // timeout ~5 m
    if (us == 0)
    {
        return 999.0; // no echo
    }
    return (us * 0.000343) / 2.0; // seconds * speed_of_sound / 2
}

/**
 * @brief RPC handler for "set_motion": commands wheel PWMs from the MPU.
 * The local ultrasonic reflex overrides this whenever halted is set, so a
 * stale or malicious command can never defeat the safety stop.
 *
 * @param left_pwm Signed PWM for the physically left wheel pair, in
 * [-255, 255].
 * @param right_pwm Signed PWM for the physically right wheel pair, in
 * [-255, 255].
 */
void rpc_set_motion(int left_pwm, int right_pwm)
{
    if (halted)
    {
        drive(0, 0);
        return;
    }
    drive(left_pwm, right_pwm);
}

/**
 * @brief RPC handler for "stop": immediately halts both motors.
 */
void rpc_stop()
{
    drive(0, 0);
}

/**
 * @brief RPC handler for "get_range": returns the current ultrasonic reading
 * for MPU-side telemetry/backup. The STM32 already enforces its own stop
 * locally; this does not gate motion by itself.
 *
 * @return Distance to the nearest obstacle in meters, or 999.0 if no echo.
 */
float rpc_get_range()
{
    return read_ultrasonic_m();
}
