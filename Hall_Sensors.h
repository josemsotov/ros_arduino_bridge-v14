/**
 * MOTOR-INTERFACE-V-13 HALL SENSORS
 * Smart Golf Trolley - Manejo de sensores Hall
 */
 
#ifndef HALL_SENSORS_H
#define HALL_SENSORS_H

#ifdef ENABLE_HALL_SENSORS

//===========================================================================
//==================== VARIABLES SENSORES HALL ============================
//===========================================================================

/**
 * CONTADORES DE PULSOS HALL (definidos en Core_Functions.h)
 * volatile uint16_t leftHallCount = 0;
 * volatile uint16_t rightHallCount = 0;
 */

/**
 * CONTADORES ACUMULATIVOS — declarados antes de las ISR
 * Solo se resetean con resetHallCounters() (comando 'r')
 */
volatile uint32_t leftHallTotal = 0;
volatile uint32_t rightHallTotal = 0;

/**
 * VARIABLES DE CONTROL DE TIEMPO
 */
unsigned long lastHallTimeLeft = 0, lastHallTimeRight = 0;
volatile unsigned long lastHallPulseTimeLeft = 0, lastHallPulseTimeRight = 0;
volatile unsigned long hallPulseIntervalLeft = 0, hallPulseIntervalRight = 0;
const unsigned long HALL_SPEED_UPDATE_MS = 100UL;
const unsigned long HALL_SPEED_MIN_INTERVAL_US = 6000UL;
volatile unsigned long hallSpeedPulseTimeLeft = 0, hallSpeedPulseTimeRight = 0;
volatile unsigned long hallSpeedIntervalLeft = 0, hallSpeedIntervalRight = 0;

/**
 * VELOCIDADES CALCULADAS
 */
float currentSpeedLeftHall = 0;   // RPM calculado motor izquierdo
float currentSpeedRightHall = 0;  // RPM calculado motor derecho

//===========================================================================
//==================== FUNCIONES DE INTERRUPCIÓN =========================
//===========================================================================

/**
 * RUTINAS DE SERVICIO DE INTERRUPCIÓN (ISR)
 */
uint16_t optoLeftPermilleForHallInterval(uint32_t interval_us) {
  // Mas rechazo contra rebote a baja velocidad; ventana menor a alta velocidad.
  if (interval_us >= 50000UL) return 650U;
  if (interval_us >= 27000UL) return 500U;
  if (interval_us >= 12000UL) return 450U;
  if (interval_us >= 8000UL) return 375U;
  return 350U;
}

void leftHallISR() {
  const uint32_t now = micros();
  leftHallCount++;
  leftHallTotal++;
  unsigned long speedInterval = now - hallSpeedPulseTimeLeft;
  if (hallSpeedPulseTimeLeft == 0 || speedInterval >= HALL_SPEED_MIN_INTERVAL_US) {
    if (hallSpeedPulseTimeLeft != 0 && speedInterval < 1000000UL) hallSpeedIntervalLeft = speedInterval;
    hallSpeedPulseTimeLeft = now;
  }
  #if defined(ENABLE_OPTO_ENCODERS) && defined(ENABLE_ADAPTIVE_OPTO_FILTER)
  if (lastHallPulseTimeLeft != 0) {
    uint32_t interval = now - lastHallPulseTimeLeft;
    if (interval < 1000000UL) {
      if (hallPulseIntervalLeft == 0 || interval < hallPulseIntervalLeft) {
        hallPulseIntervalLeft = interval; // acelerar: reducir filtro inmediatamente
      } else {
        hallPulseIntervalLeft = (hallPulseIntervalLeft * 3UL + interval) / 4UL;
      }
      uint16_t adaptive_permille = optoLeftPermilleForHallInterval(hallPulseIntervalLeft);
      uint32_t candidate = (hallPulseIntervalLeft * adaptive_permille) / 1000UL;
      leftOptoFilterUs = constrain(candidate, OPTO_FILTER_LEFT_MIN_US, OPTO_FILTER_MAX_US);
    }
  }
  lastHallPulseTimeLeft = now;
  #endif
}

void rightHallISR() {
  const uint32_t now = micros();
  rightHallCount++;
  rightHallTotal++;
  unsigned long speedInterval = now - hallSpeedPulseTimeRight;
  if (hallSpeedPulseTimeRight == 0 || speedInterval >= HALL_SPEED_MIN_INTERVAL_US) {
    if (hallSpeedPulseTimeRight != 0 && speedInterval < 1000000UL) hallSpeedIntervalRight = speedInterval;
    hallSpeedPulseTimeRight = now;
  }
  #if defined(ENABLE_OPTO_ENCODERS) && defined(ENABLE_ADAPTIVE_OPTO_FILTER)
  if (lastHallPulseTimeRight != 0) {
    uint32_t interval = now - lastHallPulseTimeRight;
    if (interval < 1000000UL) {
      if (hallPulseIntervalRight == 0 || interval < hallPulseIntervalRight) {
        hallPulseIntervalRight = interval; // acelerar: reducir filtro inmediatamente
      } else {
        hallPulseIntervalRight = (hallPulseIntervalRight * 3UL + interval) / 4UL;
      }
      uint32_t candidate = (hallPulseIntervalRight * OPTO_FILTER_RIGHT_HALL_PERMILLE) / 1000UL;
      rightOptoFilterUs = constrain(candidate, OPTO_FILTER_RIGHT_MIN_US, OPTO_FILTER_MAX_US);
    }
  }
  lastHallPulseTimeRight = now;
  #endif
}

//===========================================================================
//==================== FUNCIONES DE INICIALIZACIÓN =======================
//===========================================================================

/**
 * INICIALIZAR SENSORES HALL
 */
void initializeHallSensors() {
  // Configurar pines de entrada con pull-up interno
  pinMode(HALL_LEFT_MOTOR, INPUT_PULLUP);
  pinMode(HALL_RIGHT_MOTOR, INPUT_PULLUP);
  
  // Configurar interrupciones
  // Motor izquierdo: flanco RISING cuenta en direccion FWD
  // Motor derecho:   flanco FALLING cuenta en direccion FWD
  // (confirmado 2026-05-30: DIR fisica invertida en motor derecho)
  attachInterrupt(digitalPinToInterrupt(HALL_LEFT_MOTOR),  leftHallISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_RIGHT_MOTOR), rightHallISR, RISING);  // RISING confirmado 2026-05-30: motor D FWD genera flanco RISING
  
  // Inicializar contadores
  leftHallCount = 0;
  rightHallCount = 0;
  leftHallTotal = 0;
  rightHallTotal = 0;
  lastHallTimeLeft = millis();
  lastHallTimeRight = millis();
  lastHallPulseTimeLeft = 0;
  lastHallPulseTimeRight = 0;
  hallPulseIntervalLeft = 0;
  hallPulseIntervalRight = 0;
  
  DEBUG_PRINTLN("Sensores Hall inicializados");
  DEBUG_PRINT("PPR configurado: ");
  DEBUG_PRINTLN(PPR_HALL_SENSORS);
}

//===========================================================================
//==================== FUNCIONES DE VELOCIDAD ============================
//===========================================================================

float hallHybridSpeedRpm(
  uint16_t windowPulses,
  unsigned long elapsedMs,
  unsigned long pulseIntervalUs,
  unsigned long lastPulseUs,
  unsigned long nowUs,
  float previousRpm
) {
  if (lastPulseUs == 0 || pulseIntervalUs == 0) return 0.0f;

  // Tres periodos sin flanco (limitados a 0.3..1.0 s) confirman parada.
  unsigned long stopTimeoutUs = constrain(pulseIntervalUs * 3UL, 300000UL, 1000000UL);
  if ((unsigned long)(nowUs - lastPulseUs) > stopTimeoutUs) return 0.0f;

  float rawRpm;
  float alpha;
  if (windowPulses >= 3 && elapsedMs > 0) {
    // Velocidad media/alta: conteo de varios pulsos por ventana.
    rawRpm = (float)windowPulses * 60000.0f /
             ((float)PPR_HALL_SENSORS * (float)elapsedMs);
    alpha = 0.50f;
  } else {
    // Baja velocidad: periodo entre flancos evita cuantizacion 0/13.3 RPM.
    rawRpm = 60000000.0f /
             ((float)PPR_HALL_SENSORS * (float)pulseIntervalUs);
    alpha = 0.35f;
  }

  if (previousRpm <= 0.0f) return rawRpm;
  return previousRpm + alpha * (rawRpm - previousRpm);
}

/**
 * CALCULAR VELOCIDADES RPM DE LOS MOTORES
 */
void updateHallSpeeds() {
  unsigned long currentTime = millis();
  unsigned long nowUs = micros();

  if (currentTime - lastHallTimeLeft >= HALL_SPEED_UPDATE_MS) {
    noInterrupts();
    uint16_t pulsesLeft = leftHallCount;
    leftHallCount = 0;
    unsigned long intervalLeft = hallSpeedIntervalLeft;
    unsigned long pulseTimeLeft = hallSpeedPulseTimeLeft;
    interrupts();

    unsigned long timeElapsed = currentTime - lastHallTimeLeft;
    currentSpeedLeftHall = hallHybridSpeedRpm(
      pulsesLeft, timeElapsed, intervalLeft, pulseTimeLeft, nowUs,
      currentSpeedLeftHall);
    lastHallTimeLeft = currentTime;
  }

  if (currentTime - lastHallTimeRight >= HALL_SPEED_UPDATE_MS) {
    noInterrupts();
    uint16_t pulsesRight = rightHallCount;
    rightHallCount = 0;
    unsigned long intervalRight = hallSpeedIntervalRight;
    unsigned long pulseTimeRight = hallSpeedPulseTimeRight;
    interrupts();

    unsigned long timeElapsed = currentTime - lastHallTimeRight;
    currentSpeedRightHall = hallHybridSpeedRpm(
      pulsesRight, timeElapsed, intervalRight, pulseTimeRight, nowUs,
      currentSpeedRightHall);
    lastHallTimeRight = currentTime;
  }
}
/**
 * OBTENER VELOCIDAD MOTOR IZQUIERDO
 */
float getLeftHallSpeed() {
  return currentSpeedLeftHall;
}

/**
 * OBTENER VELOCIDAD MOTOR DERECHO
 */
float getRightHallSpeed() {
  return currentSpeedRightHall;
}

/**
 * OBTENER PROMEDIO DE VELOCIDADES
 */
float getAverageHallSpeed() {
  return (currentSpeedLeftHall + currentSpeedRightHall) / 2.0;
}

//===========================================================================
//==================== FUNCIONES DE UTILIDAD =============================
//===========================================================================

/**
 * REINICIAR CONTADORES HALL
 */
void resetHallCounters() {
  noInterrupts();
  leftHallCount = 0;
  rightHallCount = 0;
  leftHallTotal = 0;
  rightHallTotal = 0;
  lastHallPulseTimeLeft = 0;
  lastHallPulseTimeRight = 0;
  hallPulseIntervalLeft = 0;
  hallPulseIntervalRight = 0;
  hallSpeedPulseTimeLeft = 0;
  hallSpeedPulseTimeRight = 0;
  hallSpeedIntervalLeft = 0;
  hallSpeedIntervalRight = 0;
  interrupts();

  lastHallTimeLeft = millis();
  lastHallTimeRight = millis();
  currentSpeedLeftHall = 0;
  currentSpeedRightHall = 0;

  DEBUG_PRINTLN("Contadores Hall reiniciados");
}

/**
 * OBTENER INFORMACIÓN SENSORES HALL
 */
void printHallInfo() {
  updateHallSpeeds();
  
  DEBUG_PRINTLN("=== INFORMACIÓN SENSORES HALL ===");
  DEBUG_PRINT("Motor Izquierdo:");
  DEBUG_PRINT(currentSpeedLeftHall);
  DEBUG_PRINT(" RPM (");
  DEBUG_PRINT(leftHallCount);
  DEBUG_PRINTLN(" pulsos)");
  
  DEBUG_PRINT("Motor Derecho: ");
  DEBUG_PRINT(currentSpeedRightHall);
  DEBUG_PRINT(" RPM (");
  DEBUG_PRINT(rightHallCount);
  DEBUG_PRINTLN(" pulsos)");
  
  DEBUG_PRINT("Promedio: ");
  DEBUG_PRINT(getAverageHallSpeed());
  DEBUG_PRINTLN(" RPM");
  
  DEBUG_PRINT("PPR configurado: ");
  DEBUG_PRINTLN(PPR_HALL_SENSORS);
}

/**
 * DEBUG CONTINUO SENSORES HALL
 */
void printHallDebug() {
  static unsigned long lastHallDebug = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastHallDebug >= 500) {  // Cada 500ms
    updateHallSpeeds();
    
    DEBUG_PRINT("HALL L:");
    DEBUG_PRINT(currentSpeedLeftHall);
    DEBUG_PRINT("rpm(");
    DEBUG_PRINT(leftHallCount);
    DEBUG_PRINT("p) R:");
    DEBUG_PRINT(currentSpeedRightHall);
    DEBUG_PRINT("rpm(");
    DEBUG_PRINT(rightHallCount);
    DEBUG_PRINTLN("p)");
    
    lastHallDebug = currentTime;
  }
}

//===========================================================================
//==================== FUNCIONES DE PRUEBA ===============================
//===========================================================================

/**
 * PROBAR SENSORES HALL
 */
void testHallSensors() {
  DEBUG_PRINTLN("=== PRUEBA SENSORES HALL ===");
  DEBUG_PRINTLN("Moviendo motores por 5 segundos...");
  
  resetHallCounters();
  
  // Mover motores hacia adelante
  moveForward(20);  // PWM bajo para prueba
  
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {  // 5 segundos
    updateHallSpeeds();
    
    // Mostrar información cada segundo
    if ((millis() - startTime) % 1000 < 50) {
      DEBUG_PRINT("Tiempo: ");
      DEBUG_PRINT((millis() - startTime) / 1000);
      DEBUG_PRINT("s - IZQ:");
      DEBUG_PRINT(currentSpeedLeftHall);
      DEBUG_PRINT(" RPM, DER:");
      DEBUG_PRINT(currentSpeedRightHall);
      DEBUG_PRINTLN(" RPM");
    }
    
    delay(100);
  }
  
  // Detener motores
  stopAllMotors();
  
  DEBUG_PRINTLN("Prueba completada");
  printHallInfo();
}

#endif // ENABLE_HALL_SENSORS

#endif // HALL_SENSORS_H
