#include <Wire.h>

const uint8_t HALL_LEFT_PIN = 19;
const uint8_t HALL_RIGHT_PIN = 18;
const uint8_t OPTO_LEFT_PIN = 3;
const uint8_t OPTO_RIGHT_PIN = 2;
const uint8_t CURRENT_RIGHT_PIN = A3;
const uint8_t CURRENT_LEFT_PIN = A4;

volatile uint32_t hallLeftCount = 0;
volatile uint32_t hallRightCount = 0;
volatile uint32_t optoLeftCount = 0;
volatile uint32_t optoRightCount = 0;

uint32_t lastReport = 0;
uint32_t gpsChars = 0;
uint32_t gpsLines = 0;

void hallLeftISR() { hallLeftCount++; }
void hallRightISR() { hallRightCount++; }
void optoLeftISR() { optoLeftCount++; }
void optoRightISR() { optoRightCount++; }

void scanI2C() {
  Serial.print(F("DIAG I2C="));
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      if (found++) Serial.print(',');
      Serial.print(F("0x"));
      if (address < 16) Serial.print('0');
      Serial.print(address, HEX);
    }
  }
  if (!found) Serial.print(F("NONE"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  pinMode(HALL_LEFT_PIN, INPUT_PULLUP);
  pinMode(HALL_RIGHT_PIN, INPUT_PULLUP);
  pinMode(OPTO_LEFT_PIN, INPUT_PULLUP);
  pinMode(OPTO_RIGHT_PIN, INPUT_PULLUP);
  pinMode(CURRENT_RIGHT_PIN, INPUT);
  pinMode(CURRENT_LEFT_PIN, INPUT);
  // Primera fase de aislamiento: sin interrupciones para descartar saturacion
  // causada por una entrada de sensor ruidosa o con frecuencia extrema.
  delay(1000);
  Serial.println(F("DIAG BOOT OK MEGA2560"));
  Serial.println(F("DIAG I2C=SKIPPED"));
}

void loop() {
  while (Serial3.available()) {
    char c = Serial3.read();
    gpsChars++;
    if (c == '\n') gpsLines++;
  }

  uint32_t now = millis();
  if (now - lastReport >= 1000) {
    lastReport = now;
    noInterrupts();
    uint32_t hl = hallLeftCount;
    uint32_t hr = hallRightCount;
    uint32_t ol = optoLeftCount;
    uint32_t oright = optoRightCount;
    interrupts();
    Serial.print(F("DIAG ms=")); Serial.print(now);
    Serial.print(F(" hallL=")); Serial.print(hl);
    Serial.print(F(" hallR=")); Serial.print(hr);
    Serial.print(F(" optoL=")); Serial.print(ol);
    Serial.print(F(" optoR=")); Serial.print(oright);
    Serial.print(F(" pinHL=")); Serial.print(digitalRead(HALL_LEFT_PIN));
    Serial.print(F(" pinHR=")); Serial.print(digitalRead(HALL_RIGHT_PIN));
    Serial.print(F(" pinOL=")); Serial.print(digitalRead(OPTO_LEFT_PIN));
    Serial.print(F(" pinOR=")); Serial.print(digitalRead(OPTO_RIGHT_PIN));
    Serial.print(F(" currentL=")); Serial.print(analogRead(CURRENT_LEFT_PIN));
    Serial.print(F(" currentR=")); Serial.print(analogRead(CURRENT_RIGHT_PIN));
    Serial.print(F(" gpsChars=")); Serial.print(gpsChars);
    Serial.print(F(" gpsLines=")); Serial.println(gpsLines);
  }
}
