// Open loop motor control example
#include <SimpleFOC.h>

float targetVelocity = 50; // rad/s

// BLDC motor & driver instance
// BLDCMotor(int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance (ohms)
// - KV            - motor kv rating (rpm/V)
// - L             - motor phase inductance (H)
// BLDCMotor motor = BLDCMotor(4, 5.14, 5120, 0.000127); // end effector
// BLDCMotor motor = BLDCMotor(4, 9.4, 1140, 0.0013); // joints
BLDCMotor motor = BLDCMotor(4, 7.45, 812, 0.000754); // hall effect
// BLDCDriver3PWM(pin_pwmA, pin_pwmB, pin_pwmC, enable (optional))
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6);

// instantiate the commander
// Commander command = Commander(Serial);
// void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
// void doLimitCurrent(char* cmd) { command.scalar(&motor.current_limit, cmd); }

void setup() {
  // driver config
  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000; // 20k
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor current (provided resistance)
  motor.current_limit = 1.61;   // [Amps], hall effect
 
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();
  motor.initFOC();

  // add target command T
  // command.add('T', doTarget, "target velocity");
  // command.add('C', doLimitCurrent, "current limit");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  digitalWrite(4, HIGH); // set nSLEEP high
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  // open loop velocity movement
  // velocity control loop function
  // Either voltage, angle (in radians) or velocity based on the motor.controller
  motor.move(targetVelocity);

  // user communication
  // command.run();
}

