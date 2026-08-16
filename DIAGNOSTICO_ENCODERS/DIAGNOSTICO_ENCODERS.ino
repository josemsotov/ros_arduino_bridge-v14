const uint8_t OPTO_LEFT_PIN = 3;
const uint8_t OPTO_RIGHT_PIN = 2;
const uint8_t HALL_LEFT_PIN = 19;
const uint8_t HALL_RIGHT_PIN = 18;

volatile uint32_t optoLeftEdges = 0;
volatile uint32_t optoRightEdges = 0;
volatile uint32_t hallLeftEdges = 0;
volatile uint32_t hallRightEdges = 0;
volatile uint8_t optoLeftSawLow = 0;
volatile uint8_t optoLeftSawHigh = 0;
volatile uint8_t optoRightSawLow = 0;
volatile uint8_t optoRightSawHigh = 0;

void optoLeftISR() {
  optoLeftEdges++;
  digitalRead(OPTO_LEFT_PIN) ? optoLeftSawHigh = 1 : optoLeftSawLow = 1;
}
void optoRightISR() {
  optoRightEdges++;
  digitalRead(OPTO_RIGHT_PIN) ? optoRightSawHigh = 1 : optoRightSawLow = 1;
}
void hallLeftISR() { hallLeftEdges++; }
void hallRightISR() { hallRightEdges++; }

void setup() {
  Serial.begin(115200);
  pinMode(OPTO_LEFT_PIN, INPUT_PULLUP);
  pinMode(OPTO_RIGHT_PIN, INPUT_PULLUP);
  pinMode(HALL_LEFT_PIN, INPUT_PULLUP);
  pinMode(HALL_RIGHT_PIN, INPUT_PULLUP);
  optoLeftSawLow = digitalRead(OPTO_LEFT_PIN) == LOW;
  optoLeftSawHigh = digitalRead(OPTO_LEFT_PIN) == HIGH;
  optoRightSawLow = digitalRead(OPTO_RIGHT_PIN) == LOW;
  optoRightSawHigh = digitalRead(OPTO_RIGHT_PIN) == HIGH;
  attachInterrupt(digitalPinToInterrupt(OPTO_LEFT_PIN), optoLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OPTO_RIGHT_PIN), optoRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_LEFT_PIN), hallLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RIGHT_PIN), hallRightISR, CHANGE);
  delay(500);
  Serial.println(F("ENCODER_DIAG_READY CHANGE D3=LEFT D2=RIGHT"));
}

void loop() {
  static uint32_t lastReport = 0;
  if (millis() - lastReport < 250) return;
  lastReport = millis();
  noInterrupts();
  uint32_t ol = optoLeftEdges, oright = optoRightEdges;
  uint32_t hl = hallLeftEdges, hr = hallRightEdges;
  uint8_t oll = optoLeftSawLow, olh = optoLeftSawHigh;
  uint8_t orl = optoRightSawLow, orh = optoRightSawHigh;
  interrupts();
  Serial.print(F("ENC OL=")); Serial.print(ol);
  Serial.print(F(" OR=")); Serial.print(oright);
  Serial.print(F(" HL=")); Serial.print(hl);
  Serial.print(F(" HR=")); Serial.print(hr);
  Serial.print(F(" D3=")); Serial.print(digitalRead(OPTO_LEFT_PIN));
  Serial.print(F(" D2=")); Serial.print(digitalRead(OPTO_RIGHT_PIN));
  Serial.print(F(" D3_LOW=")); Serial.print(oll);
  Serial.print(F(" D3_HIGH=")); Serial.print(olh);
  Serial.print(F(" D2_LOW=")); Serial.print(orl);
  Serial.print(F(" D2_HIGH=")); Serial.println(orh);
}
