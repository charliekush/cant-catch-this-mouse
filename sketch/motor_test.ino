/**
 * @file motor_test.ino
 * @brief Standalone MCU-only sketch to sanity-check TB6612 motor wiring and
 * direction on the UNO Q.
 * @date 2026-07-14
 */

// Pin assignments below are UNVERIFIED. They come from the stock ELEGOO Smart
// Robot Car V4.0 wiring for the original UNO R3 and have not been confirmed
// against the UNO Q's actual GPIO mapping. Run this sketch, observe the robot,
// and correct these constants (and the direction polarity in set_side()) to
// match reality before trusting them anywhere else. This sketch does not use
// the Bridge, Serial1, or any router-reserved resource, so it is safe to flash
// and run standalone.

// Left motor (TB6612 channel A)
const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 8;

// Right motor (TB6612 channel B)
const int PWMB = 6;
const int BIN1 = 9;
const int BIN2 = 11;

// TB6612 standby, active HIGH
const int STBY = 3;

// Test tuning constants
const int TEST_PWM = 150;            // PWM magnitude for every move, 0-255
const unsigned long MOVE_MS = 1000;  // duration of each straight leg
const unsigned long PAUSE_MS = 1000; // pause between maneuvers
const unsigned long PIVOT_MS =
    500; // duration of each ~90 degree pivot; UNVERIFIED, tune by observation

/**
 * @brief Configures the TB6612 direction and PWM pins, enables the driver, and
 * runs the bring-up sequence once.
 */
void setup()
{
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    Serial.begin(9600);
    run_test_sequence();
}

/**
 * @brief Idles the motors. The test sequence runs once from setup(); loop()
 * only guarantees the robot stays stopped afterward.
 */
void loop()
{
    stop_motors();
}

/**
 * @brief Drives one TB6612 channel at a signed PWM value.
 *
 * @param pwm_pin PWM pin for this channel.
 * @param in1 First direction pin for this channel.
 * @param in2 Second direction pin for this channel.
 * @param value Signed PWM magnitude in [-255, 255]. Positive is assumed forward
 * and negative is assumed reverse, but this polarity is UNVERIFIED per motor.
 */
void set_side(int pwm_pin, int in1, int in2, int value)
{
    bool forward = value >= 0;
    digitalWrite(in1, forward ? HIGH : LOW);
    digitalWrite(in2, forward ? LOW : HIGH);
    int magnitude = abs(value);
    if (magnitude > 255)
    {
        magnitude = 255;
    }
    analogWrite(pwm_pin, magnitude);
}

/**
 * @brief Drives both wheels at independent signed PWM values.
 *
 * @param left_pwm Signed PWM for the left wheel, in [-255, 255].
 * @param right_pwm Signed PWM for the right wheel, in [-255, 255].
 */
void drive(int left_pwm, int right_pwm)
{
    set_side(PWMA, AIN1, AIN2, left_pwm);
    set_side(PWMB, BIN1, BIN2, right_pwm);
}

/**
 * @brief Stops both wheels.
 */
void stop_motors()
{
    drive(0, 0);
}

/**
 * @brief Runs the fixed bring-up maneuver once: forward, pivot right, forward,
 * pivot left, then halt. Logs each step over USB Serial so it can be matched to
 * what the robot physically does.
 *
 * Pivot right is assumed to be left-wheel-forward, right-wheel-reverse (and
 * pivot left the opposite); this assumption, like the pin map, is UNVERIFIED.
 */
void run_test_sequence()
{
    Serial.println("forward");
    drive(TEST_PWM, TEST_PWM);
    delay(MOVE_MS);

    Serial.println("stop");
    stop_motors();
    delay(PAUSE_MS);

    Serial.println("pivot right");
    drive(TEST_PWM, -TEST_PWM);
    delay(PIVOT_MS);

    Serial.println("stop");
    stop_motors();
    delay(PAUSE_MS);

    Serial.println("forward");
    drive(TEST_PWM, TEST_PWM);
    delay(MOVE_MS);

    Serial.println("stop");
    stop_motors();
    delay(PAUSE_MS);

    Serial.println("pivot left");
    drive(-TEST_PWM, TEST_PWM);
    delay(PIVOT_MS);

    Serial.println("stop");
    stop_motors();
    delay(PAUSE_MS);

    Serial.println("halt");
    stop_motors();
}
