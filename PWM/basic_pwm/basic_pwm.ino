#include <PWM.h>

int outPWM = 3;
int32_t frequency = 20000;

//6 is min - 1.98% duty

void setup() {
  // put your setup code here, to run once:
  InitTimersSafe();
  SetPinFrequencySafe(outPWM, frequency);
  pwmWrite(outPWM, 6);

}

void loop() {
  // put your main code here, to run repeatedly:

}
