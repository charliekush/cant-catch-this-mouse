/*
 * Evasion Bot -- STM32 side (Zephyr under the Arduino sketch layer).
 *
 * Responsibilities:
 *   - Drive the TB6612 motor driver (PWM + direction) for both wheels.
 *   - Read the kit ultrasonic sensor as a fast LOCAL safety reflex.
 *   - Expose RPC handlers the MPU calls: set_motion, stop, get_range.
 *
 * Safety principle: the ultrasonic stop is enforced HERE, locally, so a slow
 * vision frame on the MPU can never cause a head-on collision. The LiDAR does
 * the smart spatial reasoning on the MPU; this is the dumb, fast backstop.
 *
 * NOTE: Confirm the ELEGOO-chassis-to-UNO-Q pin mapping (TB6612 control pins,
 * ultrasonic trig/echo) before wiring these up. Bridge RPC registration names
 * below are PLACEHOLDERS -- verify against the real Arduino_RouterBridge API.
 */

// #include <Arduino_RouterBridge.h>   // verify actual header

// ---- Pin map (VERIFY against UNO Q GPIO the STM32 can drive) ----
const int PWMA = 5, AIN1 = 7, AIN2 = 8;    // left motor
const int PWMB = 6, BIN1 = 9, BIN2 = 11;   // right motor
const int STBY = 3;                        // TB6612 standby
const int TRIG = 12, ECHO = 13;            // ultrasonic

const float STOP_DISTANCE_M = 0.15;        // hard local halt threshold

volatile bool halted = false;

void setup() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);

  // Bridge.begin();
  // Bridge.provide("set_motion", rpc_set_motion);   // (verify API)
  // Bridge.provide("stop",       rpc_stop);
  // Bridge.provide("get_range",  rpc_get_range);
}

void loop() {
  // Fast local safety reflex, independent of the MPU.
  float d = read_ultrasonic_m();
  if (d < STOP_DISTANCE_M) {
    halted = true;
    drive(0, 0);
  } else {
    halted = false;
  }
  // Bridge.poll();   // service pending RPCs (verify API)
}

// ---- Motor control ----
void drive(int left_pwm, int right_pwm) {
  set_side(PWMA, AIN1, AIN2, left_pwm);
  set_side(PWMB, BIN1, BIN2, right_pwm);
}

void set_side(int pwm_pin, int in1, int in2, int value) {
  bool forward = value >= 0;
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  int mag = abs(value);
  if (mag > 255) mag = 255;
  analogWrite(pwm_pin, mag);
}

// ---- Ultrasonic ----
float read_ultrasonic_m() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 30000);   // timeout ~5 m
  if (us == 0) return 999.0;              // no echo
  return (us * 0.000343) / 2.0;           // seconds*speed_of_sound / 2
}

// ---- RPC handlers (placeholders; wire to verified Bridge API) ----
void rpc_set_motion(int left_pwm, int right_pwm) {
  if (halted) { drive(0, 0); return; }    // reflex overrides commands
  drive(left_pwm, right_pwm);
}

void rpc_stop() { drive(0, 0); }

float rpc_get_range() { return read_ultrasonic_m(); }
