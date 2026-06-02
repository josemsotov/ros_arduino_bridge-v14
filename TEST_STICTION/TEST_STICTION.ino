// TEST_STICTION.ino
// Incrementa PWM de 10 en 5 hasta que el motor DERECHO muestre pulsos Hall.
// Condicion: si R=0 pulsos en la ventana -> ambos motores se detienen y se sube PWM.
// Ventana de prueba: 500ms por nivel. Se cancela al primer pulso de R.

const int PWM_LEFT   = 44;
const int DIR_LEFT   = 30;
const int BRAKE_LEFT = 28;
const int STOP_LEFT  = 26;

const int PWM_RIGHT   = 46;
const int DIR_RIGHT   = 52;
const int BRAKE_RIGHT = 50;
const int STOP_RIGHT  = 48;

const int HALL_LEFT  = 19;  // INT4
const int HALL_RIGHT = 18;  // INT5

volatile uint32_t hallL = 0;
volatile uint32_t hallR = 0;

void isr_left()  { hallL++; }
void isr_right() { hallR++; }

void setup() {
  Serial.begin(115200);

  // DIR
  pinMode(DIR_LEFT,  OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  digitalWrite(DIR_LEFT,  HIGH);   // FWD izquierdo
  digitalWrite(DIR_RIGHT, LOW);    // FWD derecho (invertido electrico)

  // BRAKE: INPUT float = freno liberado
  pinMode(BRAKE_LEFT,  INPUT);
  pinMode(BRAKE_RIGHT, INPUT);

  // STOP: INPUT float = motor habilitado
  pinMode(STOP_LEFT,  INPUT);
  pinMode(STOP_RIGHT, INPUT);

  // Hall
  pinMode(HALL_LEFT,  INPUT_PULLUP);
  pinMode(HALL_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_LEFT),  isr_left,  RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_RIGHT), isr_right, RISING);

  delay(500);
  Serial.println(F("=== TEST STICTION ==="));
  Serial.println(F("Buscando PWM minimo donde motor DERECHO gira."));
  Serial.println(F("Ventana: 500ms por nivel. Si R=0 -> ambos paran."));
  Serial.println(F("PWM    L_pulsos  R_pulsos  resultado"));
  Serial.println(F("-------------------------------------"));
}

void setMotors(int pwm) {
  analogWrite(PWM_LEFT,  pwm);
  analogWrite(PWM_RIGHT, pwm);
}

void stopMotors() {
  analogWrite(PWM_LEFT,  0);
  analogWrite(PWM_RIGHT, 0);
}

void loop() {
  for (int pwm = 10; pwm <= 80; pwm += 5) {
    // Reset contadores
    noInterrupts();
    hallL = 0;
    hallR = 0;
    interrupts();

    // Arrancar ambos motores al mismo tiempo
    setMotors(pwm);

    // Monitorear durante max 500ms; abortar en cuanto R tenga pulsos
    unsigned long t0 = millis();
    bool rMoved = false;
    while (millis() - t0 < 500) {
      noInterrupts();
      uint32_t rf = hallR;
      interrupts();
      if (rf > 0) { rMoved = true; break; }
      delay(10);
    }

    // Parar SIEMPRE antes de reportar (si R no se movio, L tambien para)
    stopMotors();

    noInterrupts();
    uint32_t lf = hallL;
    uint32_t rf = hallR;
    interrupts();

    // Imprimir fila
    Serial.print(F("PWM="));
    Serial.print(pwm);
    if (pwm < 10)  Serial.print(F("  "));
    else if (pwm < 100) Serial.print(F(" "));

    Serial.print(F("  L="));
    Serial.print(lf);
    Serial.print(F("\t R="));
    Serial.print(rf);
    Serial.print(F("\t -> "));

    if (rMoved) {
      Serial.println(F("DERECHO GIRA *** UMBRAL ENCONTRADO ***"));
      Serial.println(F("-------------------------------------"));
      Serial.print(F(">>> PWM MINIMO MOTOR DERECHO = "));
      Serial.println(pwm);
      Serial.print(F("    Pulsos en 500ms: L="));
      Serial.print(lf);
      Serial.print(F("  R="));
      Serial.println(rf);
      Serial.println(F("FIN — reinicia para repetir."));
      while (true) {}
    } else {
      Serial.println(F("R sin pulsos — subiendo PWM"));
    }

    delay(400);  // pausa entre niveles
  }

  // Si llego aqui, PWM=80 y R nunca se movio
  Serial.println(F("-------------------------------------"));
  Serial.println(F("ERROR: Motor derecho sin pulsos hasta PWM=80."));
  Serial.println(F("Verificar mecanica / carga / cableado Hall."));
  while (true) {}
}
