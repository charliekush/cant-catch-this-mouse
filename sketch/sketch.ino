/**
 * @file sketch.ino
 * @brief Evasion Bot STM32 side: drives the TB6612 motor driver and exposes
 * RPC handlers to the MPU over the Bridge, with a local ultrasonic safety
 * reflex that overrides MPU motion commands independent of the Bridge link.
 * @date 2026-07-14
 */

#include "Arduino_RouterBridge.h"

#include "motors.h"




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
    motors_begin();
    Bridge.begin();
    Bridge.provide_safe("set_motion", rpc_set_motion);
    Bridge.provide_safe("stop", rpc_stop);
    //Bridge.provide_safe("get_range", rpc_get_range);
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
    
    //! TODO: implement distance tracking with lidar:
    /*
    if (d < STOP_DISTANCE_M)
    {
        halted = true;
        drive(0, 0);
    }
    else
    {
        halted = false;
    }
    */
   delay(50);
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
        motors_drive(0, 0);
        return;
    }
    motors_drive(left_pwm, right_pwm);
}

/**
 * @brief RPC handler for "stop": immediately halts both motors.
 */
void rpc_stop()
{
    motors_drive(0, 0);
}

