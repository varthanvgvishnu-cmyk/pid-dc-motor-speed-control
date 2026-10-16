
#include <Arduino.h>

// ---------- PIN CONFIGURATION ----------
const byte POT_PIN  = A0;
const byte ENA_PIN  = 5;   // PWM output
const byte IN1_PIN  = 7;
const byte IN2_PIN  = 8;
const byte HALL_PIN = 2;   // External interrupt

// ---------- SENSOR CONFIGURATION ----------
// One magnet producing one pulse per revolution.
const float PULSES_PER_REV = 1.0;

// Adjust after measuring your motor's actual speed.
const float MAX_TARGET_RPM = 300.0;

// ---------- PID SETTINGS ----------
// Initial tuning values; tune on your real motor.
float Kp = 0.8;
float Ki = 0.4;
float Kd = 0.0;

// ---------- CONTROL SETTINGS ----------
const unsigned long CONTROL_INTERVAL_MS = 100;
const unsigned long SENSOR_TIMEOUT_US = 1000000UL;
const int MIN_PWM = 0;
const int MAX_PWM = 255;

volatile unsigned long lastPulseUs = 0;
volatile unsigned long pulsePeriodUs = 0;
volatile bool pulseSeen = false;

float targetRPM = 0.0;
float actualRPM = 0.0;
float errorRPM = 0.0;
float integral = 0.0;
float previousError = 0.0;
float derivative = 0.0;
float pidOutput = 0.0;

unsigned long lastControlMs = 0;
unsigned long lastPrintMs = 0;

int pwmOutput = 0;

// ---------- HALL SENSOR INTERRUPT ----------
void hallISR() {
  unsigned long now = micros();

  if (lastPulseUs != 0) {
    unsigned long period = now - lastPulseUs;

    // Reject implausibly fast noise pulses.
    if (period > 500) {
      pulsePeriodUs = period;
      pulseSeen = true;
      lastPulseUs = now;
    }
  } else {
    lastPulseUs = now;
  }
}

// ---------- READ ACTUAL RPM ----------
float readActualRPM() {
  unsigned long period;
  unsigned long lastPulse;
  bool seen;

  noInterrupts();
  period = pulsePeriodUs;
  lastPulse = lastPulseUs;
  seen = pulseSeen;
  interrupts();

  unsigned long now = micros();

  if (!seen || period == 0 ||
      (unsigned long)(now - lastPulse) > SENSOR_TIMEOUT_US) {
    return 0.0;
  }

  return 60000000.0 / (period * PULSES_PER_REV);
}

// ---------- MOTOR CONTROL ----------
void setMotorPWM(int pwm) {
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);

  if (pwm == 0) {
    analogWrite(ENA_PIN, 0);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  } else {
    // Fixed forward direction
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, pwm);
  }
}

// ---------- PID CALCULATION ----------
void updatePID(float dt) {
  errorRPM = targetRPM - actualRPM;

  if (targetRPM < 5.0) {
    integral = 0.0;
    previousError = 0.0;
    derivative = 0.0;
    pidOutput = 0.0;
    pwmOutput = 0;
    setMotorPWM(0);
    return;
  }

  integral += errorRPM * dt;

  // Limit integral accumulation.
  integral = constrain(integral, -300.0, 300.0);

  derivative = (errorRPM - previousError) / dt;

  pidOutput = (Kp * errorRPM)
            + (Ki * integral)
            + (Kd * derivative);

  pwmOutput = (int)constrain(pidOutput, 0.0, 255.0);

  setMotorPWM(pwmOutput);

  previousError = errorRPM;
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);

  pinMode(POT_PIN, INPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Many Hall modules provide a digital output.
  pinMode(HALL_PIN, INPUT_PULLUP);

  setMotorPWM(0);

  attachInterrupt(
    digitalPinToInterrupt(HALL_PIN),
    hallISR,
    FALLING
  );

  lastControlMs = millis();
  lastPrintMs = millis();

  Serial.println("PID DC Motor Speed Control");
  Serial.println("TargetRPM,ActualRPM,ErrorRPM,PWM");
}

// ---------- MAIN LOOP ----------
void loop() {
  unsigned long nowMs = millis();

  if (nowMs - lastControlMs >= CONTROL_INTERVAL_MS) {
    float dt = (nowMs - lastControlMs) / 1000.0;
    lastControlMs = nowMs;

    int potValue = analogRead(POT_PIN);

    targetRPM =
      (potValue / 1023.0) * MAX_TARGET_RPM;

    actualRPM = readActualRPM();

    updatePID(dt);
  }

  // Print actual measured values for monitoring.
  if (nowMs - lastPrintMs >= 250) {
    lastPrintMs = nowMs;

    Serial.print(targetRPM, 1);
    Serial.print(",");
    Serial.print(actualRPM, 1);
    Serial.print(",");
    Serial.print(errorRPM, 1);
    Serial.print(",");
    Serial.println(pwmOutput);
  }
}
