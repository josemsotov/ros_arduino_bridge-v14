/**
 * TEST_MOTORES_DIAGNOSTICO
 * ============================================================
 * Sketch de diagnóstico - FASE 2
 *
 * OBJETIVO: Determinar experimentalmente qué conjunto de pines
 *           controla cada rueda y cuál DIR=HIGH/LOW es ADELANTE.
 *
 * PROCEDIMIENTO:
 *   Envía los comandos de abajo por Serial Monitor.
 *   Para cada prueba observa:
 *     - ¿Qué rueda física giró? (izquierda o derecha)
 *     - ¿En qué dirección? (adelante o atrás)
 *     - ¿Qué contador Hall aumentó? (IZQ o DER)
 *   Anota los resultados y se corregirá el firmware.
 *
 * PINES USADOS (del firmware Motor-Interface-V13):
 *   Motor A  → PWM=44  DIR=30  BRAKE=28  STOP=26
 *   Motor B  → PWM=46  DIR=52  BRAKE=50  STOP=48
 *   Hall IZQ → Pin 19 (INT4)
 *   Hall DER → Pin 18 (INT5)
 *
 * COMANDOS DISPONIBLES:
 * ─────────────────────────────────────────────────────────────
 *  MA_H   → Motor A, DIR=HIGH, PWM=40 (2 segundos)
 *  MA_L   → Motor A, DIR=LOW,  PWM=40 (2 segundos)
 *  MB_H   → Motor B, DIR=HIGH, PWM=40 (2 segundos)
 *  MB_L   → Motor B, DIR=LOW,  PWM=40 (2 segundos)
 *  STOP   → Detener todo inmediatamente
 *  STATUS → Mostrar contadores Hall actuales
 *  RESET  → Reiniciar contadores Hall a 0
 *  HELP   → Mostrar este menú
 * ─────────────────────────────────────────────────────────────
 * NOTA: Cada comando de motor resetea los contadores Hall
 *       automáticamente para medir solo ese movimiento.
 * ============================================================
 */

// ── Pines Motor A (firmware: "izquierdo") ───────────────────
#define PWM_A    44
#define DIR_A    30
#define BRAKE_A  28
#define STOP_A   26

// ── Pines Motor B (firmware: "derecho") ─────────────────────
#define PWM_B    46
#define DIR_B    52
#define BRAKE_B  50
#define STOP_B   48

// ── Pines Hall ───────────────────────────────────────────────
#define HALL_IZQ  19   // INT4
#define HALL_DER  18   // INT5

// ── Parámetros de prueba ─────────────────────────────────────
#define PWM_TEST        40     // PWM fijo para todas las pruebas
#define DURACION_MS    2000    // Duración de cada prueba en ms

// ── Contadores Hall ──────────────────────────────────────────
volatile unsigned long hallIZQ = 0;
volatile unsigned long hallDER = 0;

// ── Buffer serial ────────────────────────────────────────────
String cmdBuf = "";

// ============================================================
// ISR
// ============================================================
void ISR_IZQ() { hallIZQ++; }
void ISR_DER() { hallDER++; }

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // ── Pines Motor A ──
  pinMode(PWM_A,   OUTPUT); analogWrite(PWM_A, 0);
  pinMode(DIR_A,   OUTPUT); digitalWrite(DIR_A, LOW);
  pinMode(BRAKE_A, OUTPUT); digitalWrite(BRAKE_A, HIGH);  // freno activo
  pinMode(STOP_A,  OUTPUT); digitalWrite(STOP_A, LOW);    // deshabilitado

  // ── Pines Motor B ──
  pinMode(PWM_B,   OUTPUT); analogWrite(PWM_B, 0);
  pinMode(DIR_B,   OUTPUT); digitalWrite(DIR_B, LOW);
  pinMode(BRAKE_B, OUTPUT); digitalWrite(BRAKE_B, HIGH);  // freno activo
  pinMode(STOP_B,  OUTPUT); digitalWrite(STOP_B, LOW);    // deshabilitado

  // ── Hall ──
  pinMode(HALL_IZQ, INPUT_PULLUP);
  pinMode(HALL_DER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_IZQ), ISR_IZQ, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_DER), ISR_DER, RISING);

  Serial.println();
  Serial.println("============================================");
  Serial.println("  TEST MOTORES DIAGNOSTICO - FASE 2        ");
  Serial.println("============================================");
  Serial.println("Motor A: PWM=44 DIR=30 BRAKE=28 STOP=26");
  Serial.println("Motor B: PWM=46 DIR=52 BRAKE=50 STOP=48");
  Serial.println("Hall IZQ: pin 19 | Hall DER: pin 18");
  Serial.println();
  Serial.println("Comandos: MA_H | MA_L | MB_H | MB_L");
  Serial.println("          STOP | STATUS | RESET | HELP");
  Serial.println("--------------------------------------------");
  Serial.println("LISTO. Esperando comandos...");
  Serial.println();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuf.length() > 0) {
        procesarCmd(cmdBuf);
        cmdBuf = "";
      }
    } else if (c >= 32 && c <= 126) {
      cmdBuf += c;
    }
  }
}

// ============================================================
// FUNCIONES DE CONTROL
// ============================================================

/**
 * Detiene y deshabilita ambos motores completamente
 */
void detenerTodo() {
  // PWM a cero
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  // Frenos activos
  pinMode(BRAKE_A, OUTPUT); digitalWrite(BRAKE_A, HIGH);
  pinMode(BRAKE_B, OUTPUT); digitalWrite(BRAKE_B, HIGH);
  // STOP en LOW = deshabilitado
  pinMode(STOP_A, OUTPUT); digitalWrite(STOP_A, LOW);
  pinMode(STOP_B, OUTPUT); digitalWrite(STOP_B, LOW);
}

/**
 * Ejecuta una prueba de motor: activa solo UN motor por DURACION_MS,
 * luego lo detiene y reporta los pulsos Hall capturados.
 *
 * @param pwmPin    Pin PWM del motor
 * @param dirPin    Pin DIR del motor
 * @param brakePin  Pin BRAKE del motor
 * @param stopPin   Pin STOP del motor
 * @param dirVal    Valor DIR (HIGH o LOW)
 * @param label     Etiqueta para el reporte ("MA_H", etc.)
 */
void probarMotor(uint8_t pwmPin, uint8_t dirPin, uint8_t brakePin, uint8_t stopPin,
                 bool dirVal, const char* label) {

  Serial.println("--------------------------------------------");
  Serial.print("INICIANDO: ");
  Serial.println(label);
  Serial.print("  DIR="); Serial.print(dirVal ? "HIGH" : "LOW");
  Serial.print("  PWM="); Serial.println(PWM_TEST);
  Serial.println("  >>> OBSERVA QUE RUEDA SE MUEVE <<<");
  Serial.println();

  // Asegurar que el otro motor esté detenido
  detenerTodo();
  delay(100);

  // Resetear contadores Hall para esta prueba
  noInterrupts();
  hallIZQ = 0;
  hallDER = 0;
  interrupts();

  // ── Secuencia de habilitación del driver ZS-X11H ──
  // 1. Escribir DIR antes de habilitar STOP (el driver latchea DIR al habilitar)
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, dirVal);
  delay(5);

  // 2. Liberar freno
  pinMode(brakePin, OUTPUT);
  digitalWrite(brakePin, LOW);
  delay(5);

  // 3. Habilitar motor: STOP=INPUT (float = enable)
  pinMode(stopPin, INPUT);
  delay(5);

  // 4. Aplicar PWM
  analogWrite(pwmPin, PWM_TEST);

  // ── Esperar duración de la prueba ──────────────────
  unsigned long inicio = millis();
  unsigned long snapIZQ_prev = 0, snapDER_prev = 0;

  while (millis() - inicio < DURACION_MS) {
    // Mostrar pulsos cada 500ms durante el movimiento
    if ((millis() - inicio) % 500 < 20) {
      noInterrupts();
      unsigned long sIZQ = hallIZQ;
      unsigned long sDER = hallDER;
      interrupts();
      if (sIZQ != snapIZQ_prev || sDER != snapDER_prev) {
        Serial.print("  ["); Serial.print(millis() - inicio);
        Serial.print("ms] IZQ="); Serial.print(sIZQ);
        Serial.print("  DER="); Serial.println(sDER);
        snapIZQ_prev = sIZQ;
        snapDER_prev = sDER;
      }
      delay(25);  // evitar spam del mismo tick
    }
  }

  // ── Detener motor ──────────────────────────────────
  analogWrite(pwmPin, 0);
  delay(20);
  pinMode(stopPin, OUTPUT);
  digitalWrite(stopPin, LOW);    // deshabilitar
  digitalWrite(brakePin, HIGH);  // freno activo

  // ── Reporte final ──────────────────────────────────
  noInterrupts();
  unsigned long finalIZQ = hallIZQ;
  unsigned long finalDER = hallDER;
  interrupts();

  Serial.println();
  Serial.print("RESULTADO ");
  Serial.print(label);
  Serial.println(":");
  Serial.print("  Pulsos Hall IZQ = "); Serial.println(finalIZQ);
  Serial.print("  Pulsos Hall DER = "); Serial.println(finalDER);

  if (finalIZQ > finalDER && finalIZQ > 3) {
    Serial.println("  >> Hall activo: IZQ (sensor izquierdo detecto movimiento)");
  } else if (finalDER > finalIZQ && finalDER > 3) {
    Serial.println("  >> Hall activo: DER (sensor derecho detecto movimiento)");
  } else if (finalIZQ <= 3 && finalDER <= 3) {
    Serial.println("  >> ADVERTENCIA: Pocos pulsos detectados (<= 3)");
    Serial.println("     Verifica conexion Hall o aumenta PWM_TEST");
  } else {
    Serial.println("  >> Ambos sensores activos (posible rebote o conexion cruzada)");
  }

  Serial.println();
  Serial.println("ANOTA: Que rueda giro fisicamente? Adelante o atras?");
  Serial.println("--------------------------------------------");
  Serial.println();
}

// ============================================================
// PROCESADOR DE COMANDOS
// ============================================================
void procesarCmd(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "MA_H") {
    probarMotor(PWM_A, DIR_A, BRAKE_A, STOP_A, HIGH, "MA_H (Motor-A DIR=HIGH)");

  } else if (cmd == "MA_L") {
    probarMotor(PWM_A, DIR_A, BRAKE_A, STOP_A, LOW, "MA_L (Motor-A DIR=LOW)");

  } else if (cmd == "MB_H") {
    probarMotor(PWM_B, DIR_B, BRAKE_B, STOP_B, HIGH, "MB_H (Motor-B DIR=HIGH)");

  } else if (cmd == "MB_L") {
    probarMotor(PWM_B, DIR_B, BRAKE_B, STOP_B, LOW, "MB_L (Motor-B DIR=LOW)");

  } else if (cmd == "STOP") {
    detenerTodo();
    Serial.println(">> STOP: Todo detenido.");

  } else if (cmd == "STATUS") {
    noInterrupts();
    unsigned long sIZQ = hallIZQ;
    unsigned long sDER  = hallDER;
    interrupts();
    Serial.print("[HALL] IZQ="); Serial.print(sIZQ);
    Serial.print("  DER="); Serial.println(sDER);

  } else if (cmd == "RESET") {
    noInterrupts();
    hallIZQ = 0;
    hallDER = 0;
    interrupts();
    Serial.println(">> Contadores reiniciados.");

  } else if (cmd == "HELP") {
    Serial.println("============================================");
    Serial.println("COMANDOS DISPONIBLES:");
    Serial.println("  MA_H  -> Motor A (pines 44/30/28/26) DIR=HIGH, 2 seg");
    Serial.println("  MA_L  -> Motor A (pines 44/30/28/26) DIR=LOW,  2 seg");
    Serial.println("  MB_H  -> Motor B (pines 46/52/50/48) DIR=HIGH, 2 seg");
    Serial.println("  MB_L  -> Motor B (pines 46/52/50/48) DIR=LOW,  2 seg");
    Serial.println("  STOP  -> Detener todo");
    Serial.println("  STATUS-> Ver contadores Hall");
    Serial.println("  RESET -> Reiniciar contadores Hall");
    Serial.println("  HELP  -> Este menu");
    Serial.println("============================================");

  } else {
    Serial.print(">> Desconocido: "); Serial.println(cmd);
    Serial.println("   Usa HELP para ver los comandos.");
  }
}
