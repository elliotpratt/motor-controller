#include <Servo.h>

#define BAUDRATE         115200
#define NUM_BITS         10
#define MONOFLOP_TIME    25

#define E0_DATA_PIN      5
#define E0_CLOCK_PIN     6
#define E1_DATA_PIN      7  // 205.75 for closed position, main one, increase the angle to open
#define E1_CLOCK_PIN     8  // main encoder
#define ESC_PIN          9

#define TARGET_DEGREE    90
#define MIN_PULSE_WID    1000 //for ESC, in microseconds
#define MAX_PULSE_WID    2000

Servo ESC;
const unsigned int CPR = (unsigned int)pow(2, NUM_BITS);
uint16_t encoderPosition0;
uint16_t encoderPosition1;
int motorSpeed;
int potVal;

void updateVals() {
  encoderPosition0 = readPosition(E0_CLOCK_PIN, E0_DATA_PIN);
  // encoderPosition1 = readPosition(E1_CLOCK_PIN, E1_DATA_PIN);
  //FIXME
  // potVal = analogRead(A0);
  // motorSpeed = map(potVal, 15, 900, 0, 180);
  // if (motorSpeed < 0) { motorSpeed = 0; }
  // if (motorSpeed > 180) { motorSpeed = 180; }
  // ESC.write(motorSpeed);
  // Serial.println(motorSpeed);
}

void output() {
  Serial.print("Encoder 0: ");
  Serial.println(encoderPosition0, DEC);
  Serial.print("Encoder 0 in degrees: ");
  Serial.println(getEncoder0PositionDeg());

  // Serial.print("Encoder 1: ");
  // Serial.println(encoderPosition1, DEC);
  // Serial.print("Encoder 1 in degrees: ");
  // Serial.println(getEncoder1PositionDeg());
}

double getEncoder0PositionDeg() {
  return encoderPosition0 * 360.0 / CPR;
}

uint16_t getEncoder0Position() {
  return encoderPosition0;
}

double getEncoder1PositionDeg() {
  return encoderPosition1 * 360.0 / CPR;
}

uint16_t getEncoder1Position() {
  return encoderPosition1;
}

//read the current angular position
uint16_t readPosition(int clockPin, int dataPin) {
  delayMicroseconds(MONOFLOP_TIME);  // Clock must be high for 25 microseconds before a new sample can be taken

  // Read the same position data twice to check for angleDists
  uint16_t sample1 = shiftIn(clockPin, dataPin);
  delayMicroseconds(MONOFLOP_TIME);  // Clock must be high for 25 microseconds before a new sample can be taken
  uint16_t sample2 = shiftIn(clockPin, dataPin);

  if (sample1 != sample2) {
    Serial.print("Samples don't match: sample1 = ");
    Serial.print(sample1, DEC);
    Serial.print(", sample2 = ");
    Serial.println(sample2, DEC);
  }

  return sample1;
}

//read in a byte of data from the digital input of the board.
uint16_t shiftIn(int clockPin, int dataPin) {
  uint16_t data = 0;

  for (int i=0; i<NUM_BITS; ++i) {
    data <<= 1;
    digitalWrite(clockPin, LOW);
    // delayMicroseconds(1);
    asm volatile("nop \n\t""nop \n\t""nop \n\t");
    digitalWrite(clockPin, HIGH);
    // delayMicroseconds(1);
    asm volatile("nop \n\t""nop \n\t""nop \n\t");

    data |= digitalRead(dataPin);
  }

  digitalWrite(clockPin, LOW);
  // delayMicroseconds(1);
  asm volatile("nop \n\t""nop \n\t""nop \n\t");
  digitalWrite(clockPin, HIGH);

  return data;
}


void setup() {
  pinMode(E0_DATA_PIN, INPUT);
  // pinMode(E1_DATA_PIN, INPUT);
  pinMode(E0_CLOCK_PIN, OUTPUT);
  // pinMode(E1_CLOCK_PIN, OUTPUT);

  digitalWrite(E0_CLOCK_PIN, HIGH);
  // digitalWrite(E1_CLOCK_PIN, HIGH);

  Serial.begin(BAUDRATE);
  // ESC.attach(ESC_PIN, MIN_PULSE_WID, MAX_PULSE_WID);
  // ESC.write(90); //no rotation
}

void loop() {
  // read position and update
  // delay(100);
  updateVals();
  // output results
  output();
  // ESC.write(0);
}
