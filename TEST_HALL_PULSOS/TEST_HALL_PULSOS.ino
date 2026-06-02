/**
 * TEST_HALL_PULSOS
 * ============================================================
 * Sketch de diagnóstico - FASE 1
 * 
 * OBJETIVO: Verificar que los sensores Hall funcionan y que
 *           cada sensor corresponde a la rueda correcta.
 * 
 * PROCEDIMIENTO:
 *   1. Sube este sketch al Arduino Mega
 *   2. Abre el Serial Monitor a 115200 baudios
 *   3. Gira la RUEDA IZQUIERDA manualmente (a mano)
 *      → El contador IZQ debe aumentar, DER debe quedarse igual
 *   4. Envía RESET por serial para reiniciar contadores
 *   5. Gira la RUEDA DERECHA manualmente
 *      → El contador DER debe aumentar, IZQ debe quedarse igual
 *   6. Si un contador no responde o responde la rueda contraria,
 *      los pines están invertidos → reportar al firmware
 * 
 * PINES (según Pins.h del firmware principal):
 *   Pin 19 → HALL_LEFT_MOTOR  (Interrupción 4)
 *   Pin 18 → HALL_RIGHT_MOTOR (Interrupción 5)
 * 
 * COMANDOS SERIAL:
 *   RESET   → Reinicia ambos contadores a 0
 *   STATUS  → Muestra conteo actual una vez
 *   AUTO    → Activa/desactiva reporte automático cada segundo
 * ============================================================
 */

// ── Pines (mismos que Pins.h del firmware) ──────────────────
#define HALL_PIN_IZQ   19    // Interrupción 4 - Sensor Hall izquierdo
#define HALL_PIN_DER   18    // Interrupción 5 - Sensor Hall derecho

// ── Contadores de pulsos (volatile porque los modifica la ISR) ──
volatile unsigned long pulsosIZQ = 0;
volatile unsigned long pulsosDER = 0;

// ── Control de reporte automático ───────────────────────────
bool autoReport = true;
unsigned long ultimoReporte = 0;
const unsigned long INTERVALO_REPORTE_MS = 1000;  // Reportar cada 1 segundo

// ── Buffer de comando serial ─────────────────────────────────
String cmdBuffer = "";

// ============================================================
// RUTINAS DE INTERRUPCIÓN (ISR)
// ============================================================

void ISR_Hall_IZQ() {
  pulsosIZQ++;
}

void ISR_Hall_DER() {
  pulsosDER++;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // Esperar conexión (necesario en algunas placas)

  // Configurar pines como entrada con pull-up interno
  pinMode(HALL_PIN_IZQ, INPUT_PULLUP);
  pinMode(HALL_PIN_DER, INPUT_PULLUP);

  // Adjuntar interrupciones - flanco RISING (pulso positivo)
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_IZQ), ISR_Hall_IZQ, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_DER), ISR_Hall_DER, RISING);

  Serial.println();
  Serial.println("============================================");
  Serial.println("  TEST HALL PULSOS - FASE 1 DIAGNOSTICO   ");
  Serial.println("============================================");
  Serial.println("Pines configurados:");
  Serial.println("  Pin 19 → Hall IZQUIERDO (INT4)");
  Serial.println("  Pin 18 → Hall DERECHO   (INT5)");
  Serial.println();
  Serial.println("PROCEDIMIENTO:");
  Serial.println("  1. Gira la RUEDA IZQUIERDA a mano");
  Serial.println("     -> Debe aumentar solo el contador IZQ");
  Serial.println("  2. Envia RESET, luego gira la RUEDA DERECHA");
  Serial.println("     -> Debe aumentar solo el contador DER");
  Serial.println();
  Serial.println("COMANDOS: RESET | STATUS | AUTO");
  Serial.println("--------------------------------------------");
  Serial.println("Reporte automatico ACTIVO (cada 1 segundo)");
  Serial.println();
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  // ── Leer comando serial ──────────────────────────────────
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() > 0) {
        procesarComando(cmdBuffer);
        cmdBuffer = "";
      }
    } else if (c >= 32 && c <= 126) {
      cmdBuffer += c;
    }
  }

  // ── Reporte automático cada segundo ─────────────────────
  if (autoReport && (millis() - ultimoReporte >= INTERVALO_REPORTE_MS)) {
    ultimoReporte = millis();
    imprimirContadores();
  }
}

// ============================================================
// FUNCIONES
// ============================================================

/**
 * Imprime el estado actual de los contadores
 */
void imprimirContadores() {
  // Leer contadores de forma segura (deshabilitar interrupciones momentáneamente)
  noInterrupts();
  unsigned long snapIZQ = pulsosIZQ;
  unsigned long snapDER = pulsosDER;
  interrupts();

  Serial.print("[HALL] IZQ=");
  Serial.print(snapIZQ);
  Serial.print("  DER=");
  Serial.print(snapDER);
  Serial.print("  TOTAL=");
  Serial.println(snapIZQ + snapDER);
}

/**
 * Procesa un comando recibido por serial
 */
void procesarComando(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "RESET") {
    noInterrupts();
    pulsosIZQ = 0;
    pulsosDER = 0;
    interrupts();
    Serial.println(">> Contadores reiniciados a 0");

  } else if (cmd == "STATUS") {
    imprimirContadores();

  } else if (cmd == "AUTO") {
    autoReport = !autoReport;
    if (autoReport) {
      Serial.println(">> Reporte automatico ACTIVADO");
    } else {
      Serial.println(">> Reporte automatico DESACTIVADO (usa STATUS para consultar)");
    }

  } else {
    Serial.print(">> Comando desconocido: ");
    Serial.println(cmd);
    Serial.println("   Comandos validos: RESET | STATUS | AUTO");
  }
}
