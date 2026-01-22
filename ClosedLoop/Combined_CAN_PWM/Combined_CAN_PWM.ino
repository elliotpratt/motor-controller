#include <PWM.h>
#include <SPI.h>
#include <mcp_can.h>

#define BAUDRATE 115200
#define CAN_CS_PIN 10
#define CAN_INT_PIN 2

MCP_CAN CAN(CAN_CS_PIN);

// Variables from CORE
int motorDirection = 0;
int motorSpeed = 0;
int EndPosition = 0;
unsigned int MaxTime = 0;
int received = 0;
int sped = 0;

/*
  Evaluation board notes:
  dir, brake, and drvoff at 3V when on
  CCW is HIGH, CW is LOW
  off is HIGH, on is LOW
  brake is HIGH, run is LOW
  CW rotation of motor corresponds to decreasing position on encoder
*/

// properties of the encoder
const int NUM_BITS = 10;
const unsigned int CPR = (unsigned int)pow(2, NUM_BITS);
const int MONOFLOP_TIME = 25;

// pin numbers
const int MOTOR_PIN = 9;
const int DATA_PIN = 7;
const int CLOCK_PIN = 6;
const int DRVOFF_PIN = 4;
const int DIR_PIN = 8;
const int NFAULT_PIN = 3;

// other constants
const double TOLERANCE = 0.25; // feel free to change this as desired
const int32_t PWM_FREQ = 20000; // 20 kHz PWM
const int MIN_PWM = 50; // min value to write to motor to get it to actually run; 6/256 = ~2% duty cycle

// variables
int encoderPosition; // from 0 to 4095
int rawEncoderPosition;
double encoderAngle; // from 0 to 359
double rawEncoderAngle;
uint16_t zero; // encoder reading at starting position, from 0 to 4095
int currDir;
double currError;
bool timedOut = false;

volatile bool fault = false;

void nfault() {
  fault = true;
}

// print info about position and speed
void output() {
  Serial.print("Encoder position: ");
  Serial.println(encoderPosition, DEC);
  Serial.print("Encoder position in degrees: ");
  Serial.println(encoderAngle);
}

// update encoder position and encoder angle
void updatePosition() {
  // map absolute reading to a value relative to the desired zero point
  int ret = readPosition();
  if (ret == -1) return;

  rawEncoderPosition = ret;
  encoderPosition = ret - zero;
  if (encoderPosition < 0) {
    encoderPosition += CPR;
  }
  rawEncoderAngle = rawEncoderPosition * 360.0 / CPR;
  encoderAngle = encoderPosition * 360.0 / CPR;
  // convert large positive angles into small negative angles
//  if (encoderAngle > 180.0) {
//    encoderAngle -= 360.0;
//  }
}

// read the current angular position
int readPosition() {
  delayMicroseconds(MONOFLOP_TIME);  // clock must be high for 25 microseconds before a new sample can be taken
  uint16_t sample1 = shiftIn();

//  if (sample1 != sample2) {
//    Serial.println("Error!");
//    return -1;
//    // Serial.print("Samples don't match: sample1 = ");
//    // Serial.print(sample1, DEC);
//    // Serial.print(", sample2 = ");
//    // Serial.println(sample2, DEC);
//  }

  return sample1;
}

// read in 12 bits of encoder data
uint16_t shiftIn() {
  uint16_t value = 0;

  cli();  // disable interrupts for timing determinism

  for (uint8_t i = 0; i < NUM_BITS; i++) {

    // CLK low
    PORTD &= ~(1 << CLOCK_PIN);
    asm volatile("nop\nnop\nnop\nnop\nnop\nnop\n");  // ~250 ns

    // CLK high
    PORTD |= (1 << CLOCK_PIN);
    asm volatile("nop\nnop\nnop\n");

    // Sample DATA
    value <<= 1;
    value |= (PIND >> DATA_PIN) & 0x01;
  }

  sei();  // re-enable interrupts

  return value;
}

void setup() {
  // begin serial for debugging
  Serial.begin(BAUDRATE);

  // PWM setup for output to motor
  InitTimersSafe();
  SetPinFrequencySafe(MOTOR_PIN, PWM_FREQ);
  pwmWrite(MOTOR_PIN, 0);

  // set type of encoder and motor control pins
  pinMode(DATA_PIN, INPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DRVOFF_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(NFAULT_PIN, INPUT_PULLUP);

  // initialize encoder and motor control pins
  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(DRVOFF_PIN, HIGH); // start off
  digitalWrite(DIR_PIN, LOW); // start CW

  // initialize zero point for encoder
  delay(25);
  do {
    zero = readPosition();
  } while (zero == -1);
  encoderPosition = 0;
  encoderAngle = 0.0;
  currDir = LOW;
  currError = 0;

  // pwmWrite(MOTOR_PIN, 10);
  attachInterrupt(1, nfault, FALLING);
}

void loop() {
  updatePosition();
  output();
}
