/**
 * TEST_PWM_SWEEP — Barrido de PWM para calibrar umbral óptimo
 * Driver: ZS-X11H  |  Board: Arduino Mega 2560
 * Pines idénticos a MOTOR-INTERFACE-V14
 *
 * Prueba PWM 10..80 paso 5, ventana 1s por nivel.
 * Al final imprime tabla completa L/R pulsos por PWM.
 * Serial 115200
 */

// ── PINES MOTOR IZQUIERDO ──────────────────────────────────────────────────
#define PWM_LEFT    44
#define DIR_LEFT    30
#define BRAKE_LEFT  28
#define STOP_LEFT   26

// ── PINES MOTOR DERECHO ────────────────────────────────────────────────────
#define PWM_RIGHT   46
#define DIR_RIGHT   52
#define BRAKE_RIGHT 50
#define STOP_RIGHT  48

// ── HALL SENSORS (monitoreo pasivo) ──────────────────────────────────────
#define HALL_LEFT_PIN  19   // INT4
#define HALL_RIGHT_PIN 18   // INT5

volatile uint32_t hallL = 0;
volatile uint32_t hallR = 0;

void isrL() { hallL++; }
void isrR() { hallR++; }

// ── SETUP ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== TEST MINIMAL MOTORES ==="));
  Serial.println(F("Driver ZS-X11H | Mega 2560"));
  Serial.println(F("Pines L: PWM=44 DIR=30 BRAKE=28 STOP=26"));
  Serial.println(F("Pines R: PWM=46 DIR=52 BRAKE=50 STOP=48"));

  // ── Configurar pines de motor ──────────────────────────────────────────
  pinMode(PWM_LEFT,    OUTPUT);
  pinMode(DIR_LEFT,    OUTPUT);
  pinMode(PWM_RIGHT,   OUTPUT);
  pinMode(DIR_RIGHT,   OUTPUT);

  // ── Estado seguro inicial: DISABLE + BRAKE ─────────────────────────────
  analogWrite(PWM_LEFT,  0);
  analogWrite(PWM_RIGHT, 0);

  // STOP LOW (OUTPUT) = motor deshabilitado
  pinMode(STOP_LEFT,  OUTPUT);  digitalWrite(STOP_LEFT,  LOW);
  pinMode(STOP_RIGHT, OUTPUT);  digitalWrite(STOP_RIGHT, LOW);

  // BRAKE HIGH (OUTPUT) = freno activado
  pinMode(BRAKE_LEFT,  OUTPUT);  digitalWrite(BRAKE_LEFT,  HIGH);
  pinMode(BRAKE_RIGHT, OUTPUT);  digitalWrite(BRAKE_RIGHT, HIGH);

  delay(200);

  // ── Hall sensors ───────────────────────────────────────────────────────
  pinMode(HALL_LEFT_PIN,  INPUT_PULLUP);
  pinMode(HALL_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_LEFT_PIN),  isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_RIGHT_PIN), isrR, RISING);

  // ── HABILITAR MOTORES ──────────────────────────────────────────────────
  // 1. Liberar freno: BRAKE = INPUT (float) = freno desactivado
  pinMode(BRAKE_LEFT,  INPUT);
  pinMode(BRAKE_RIGHT, INPUT);

  // 2. Habilitar: STOP = INPUT (float) = motor habilitado
  pinMode(STOP_LEFT,  INPUT);
  pinMode(STOP_RIGHT, INPUT);

  // 3. Dirección FWD
  //    Izquierdo: DIR HIGH = adelante
  //    Derecho:   DIR LOW  = adelante (electricamente invertido, verificado)
  digitalWrite(DIR_LEFT,  HIGH);
  digitalWrite(DIR_RIGHT, LOW);

  Serial.println(F("Motores habilitados. Iniciando barrido PWM..."));
  Serial.println(F("Ventana 1s por nivel, pausa 500ms entre niveles."));
  Serial.println(F("----------------------------------------"));
}

// ── Tabla de resultados (PWM 10..80 paso 5 = 15 filas) ────────────────────
#define N_STEPS 15
static uint8_t  tbl_pwm[N_STEPS];
static uint32_t tbl_L[N_STEPS];
static uint32_t tbl_R[N_STEPS];

// ── LOOP ───────────────────────────────────────────────────────────────────
void loop() {
  int idx = 0;
  for (int pwm = 10; pwm <= 80; pwm += 5) {

    // Reset contadores
    noInterrupts(); hallL = 0; hallR = 0; interrupts();

    // Arrancar ambos motores
    analogWrite(PWM_LEFT,  pwm);
    analogWrite(PWM_RIGHT, pwm);

    // Ventana de medición: 1000 ms
    delay(1000);

    // Leer contadores
    noInterrupts();
    uint32_t l = hallL;
    uint32_t r = hallR;
    interrupts();

    // Parar
    analogWrite(PWM_LEFT,  0);
    analogWrite(PWM_RIGHT, 0);

    // Guardar
    tbl_pwm[idx] = pwm;
    tbl_L[idx]   = l;
    tbl_R[idx]   = r;
    idx++;

    // Log en tiempo real
    Serial.print(F("PWM=")); Serial.print(pwm);
    Serial.print(F("  L=")); Serial.print(l);
    Serial.print(F("  R=")); Serial.println(r);

    // Pausa entre niveles
    delay(500);
  }

  // ── TABLA FINAL ───────────────────────────────────────────────────────
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" TABLA RESULTADOS BARRIDO PWM"));
  Serial.println(F("========================================"));
  Serial.println(F(" PWM | L puls | R puls | L rpm* | R rpm*"));
  Serial.println(F("-----|--------|--------|--------|-------"));

  // *rpm estimados: pulsos en 1s / PPR * 60   (PPR=45)
  for (int i = 0; i < N_STEPS; i++) {
    uint8_t  p  = tbl_pwm[i];
    uint32_t l  = tbl_L[i];
    uint32_t r  = tbl_R[i];
    float    lrpm = (float)l / 45.0f * 60.0f;
    float    rrpm = (float)r / 45.0f * 60.0f;

    // Formato alineado
    char buf[50];
    snprintf(buf, sizeof(buf),
      "  %3d |  %5lu  |  %5lu  | %6.1f | %5.1f",
      (int)p, (unsigned long)l, (unsigned long)r, lrpm, rrpm);
    Serial.println(buf);
  }

  Serial.println(F("========================================"));
  Serial.println(F("FIN — reinicia el Arduino para repetir."));
  while (true) {}
}
