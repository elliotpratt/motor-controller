#include <PWM.h>

#define BAUDRATE 115200

/*
  Evaluation board notes:
  dir, brake, and drvoff at 3V when on
  CCW is HIGH, CW is LOW
  off is HIGH, on is LOW
  brake is HIGH, run is LOW
  CW rotation of motor corresponds to decreasing position on encoder
*/

// pin numbers
const int MOTOR_PIN = 9;
const int NSLEEP_PIN = 5;
const int NFAULT_PIN = 3;

// other constants
const int32_t PWM_FREQ = 20000; // 20 kHz PWM
const int MIN_PWM = 50; // min value to write to motor to get it to actually run; 6/256 = ~2% duty cycle

volatile bool fault = false;

void nfault() {
  fault = true;
}

void setup() {
  // begin serial for debugging
  Serial.begin(BAUDRATE);

  // PWM setup for output to motor
  InitTimersSafe();
  SetPinFrequencySafe(MOTOR_PIN, PWM_FREQ);

  // set type of encoder and motor control pins
  pinMode(NSLEEP_PIN, OUTPUT);
  pinMode(NFAULT_PIN, INPUT_PULLUP);

  // set sleep high
  digitalWrite(NSLEEP_PIN, HIGH); // start on

  attachInterrupt(1, nfault, FALLING);

  // pwmWrite(MOTOR_PIN, 10);
  pwmWrite(MOTOR_PIN, 150);
}

void loop() {
  if (fault) {
    Serial.println("Fault detected");
    pwmWrite(MOTOR_PIN, 0);
  }
  fault = false;
  pwmWrite(MOTOR_PIN, 150);
}
