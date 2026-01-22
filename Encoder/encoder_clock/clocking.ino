// SSI pins on PORTD
#define SSI_CLK_BIT   6   // PD6 (D6)
#define SSI_DATA_BIT  7   // PD7 (D7)

uint16_t readSSI12() {
  uint16_t value = 0;

  cli();  // disable interrupts for timing determinism

  for (uint8_t i = 0; i < 12; i++) {

    // CLK low
    PORTD &= ~(1 << SSI_CLK_BIT);
    asm volatile("nop\nnop\nnop\nnop\n");  // ~250 ns

    // CLK high
    PORTD |= (1 << SSI_CLK_BIT);
    asm volatile("nop\nnop\nnop\n");

    // Sample DATA
    value <<= 1;
    value |= (PIND >> SSI_DATA_BIT) & 0x01;
  }

  sei();  // re-enable interrupts

  return value;
}

void setup() {
  DDRD  |=  (1 << SSI_CLK_BIT);   // CLK as output
  DDRD  &= ~(1 << SSI_DATA_BIT);  // DATA as input
  PORTD |= (1 << SSI_CLK_BIT);   // CLK high
  Serial.begin(115200);  
}

void loop() {
  uint16_t result = readSSI12();
  delayMicroseconds(50);
  Serial.println(result);
}