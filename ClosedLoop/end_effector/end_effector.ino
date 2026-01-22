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

// properties of the encoder
const int NUM_BITS = 12;
const unsigned int CPR = (unsigned int)pow(2, NUM_BITS);
const int MONOFLOP_TIME = 25;

// pin numbers
const int MOTOR_PIN = 3;
const int DATA_PIN = 5;
const int CLOCK_PIN = 6;
const int DRVOFF_PIN = 7;
const int DIR_PIN = 8;

// other constants
const double TOLERANCE = 2.0; // feel free to change this as desired
const int32_t PWM_FREQ = 20000; // 20 kHz PWM
const int MIN_PWM = 6; // min value to write to motor to get it to actually run; 6/256 = ~2% duty cycle

// variables
int encoderPosition; // from 0 to 4095
int rawEncoderPosition;
double encoderAngle; // from 0 to 359
double rawEncoderAngle;
uint16_t zero; // encoder reading at starting position, from 0 to 4095
int currDir;
double currError;

// print info about position and speed
void output() {
  Serial.print("Encoder position: ");
  Serial.println(encoderPosition, DEC);
  Serial.print("Encoder position in degrees: ");
  Serial.println(encoderAngle);
}

// move end effector to target position
void moveToAngle(double target) {
  // for dir, 0 is CW, 1 is CCW
  double error;
  int dir;

  do {
    updatePosition();
    error = abs(target - encoderAngle); // calculate absolute distance from target angle
    Serial.print("Encoder angle: ");
    Serial.println(encoderAngle);
    Serial.print("Raw encoder angle: ");
    Serial.println(rawEncoderAngle);
    Serial.print("Error: ");
    Serial.println(error);
    dir = (encoderAngle > target) ? LOW : HIGH;
    // don't need to rewrite if nothing has changed
    if (error != currError || dir != currDir) {
      move(dir, error);
    }
  } while(error > TOLERANCE);

  // ensure signal is zero before returning
  pwmWrite(MOTOR_PIN, 0);
}

// move motor according to error between current position and target position
void move(int dir, double error) {
  digitalWrite(DIR_PIN, dir);
  pwmWrite(MOTOR_PIN, MIN_PWM);
  currDir = dir;
  currError = error;
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
  if (encoderAngle > 200.0) {
    encoderAngle -= 360.0;
  }
}

// read the current angular position
int readPosition() {
  uint16_t sample1 = shiftIn();
  uint16_t sample2 = shiftIn();

  delayMicroseconds(MONOFLOP_TIME);  // clock must be high for 25 microseconds before a new sample can be taken

  if (sample1 != sample2) {
    Serial.println("Error!");
    return -1;
    // Serial.print("Samples don't match: sample1 = ");
    // Serial.print(sample1, DEC);
    // Serial.print(", sample2 = ");
    // Serial.println(sample2, DEC);
  }

  return sample1;
}

// read in 12 bits of encoder data
uint16_t shiftIn() {
  uint16_t data = 0;

  for (int i = 0; i < NUM_BITS; ++i) {
    data <<= 1;
    // send clock pulse to interrogate encoder
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1);

    data |= digitalRead(DATA_PIN); // read data bit
  }

  digitalWrite(CLOCK_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(CLOCK_PIN, HIGH);

  return data;
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

  pwmWrite(MOTOR_PIN, 50);
}

void loop() {
  updatePosition();
  output();
  // delay(10);

  // moveToAngle(85);
  // Serial.println("Finished going to 85");
  // delay(10000);

  // moveToAngle(45);
  // output();
  // delay(2000);

  // moveToAngle(5);
  // Serial.println("Finished going to 5");
  // delay(10000);

  // moveToAngle(55);
  // output();
  // delay(2000);
}