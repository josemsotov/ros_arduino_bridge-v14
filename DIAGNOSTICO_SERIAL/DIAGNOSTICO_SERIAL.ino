void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("SERIAL_BOOT_OK");
}

void loop() {
  Serial.print("SERIAL_HEARTBEAT ");
  Serial.println(millis());
  delay(1000);
}
