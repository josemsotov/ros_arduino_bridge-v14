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
unsigned long lastHallPulseTimeLeft = 0, lastHallPulseTimeRight = 0;
volatile unsigned long hallPulseIntervalLeft = 0, hallPulseIntervalRight = 0;

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
void leftHallISR() {
  const uint32_t now = micros();
  leftHallCount++;
  leftHallTotal++;
  #if defined(ENABLE_OPTO_ENCODERS) && defined(ENABLE_ADAPTIVE_OPTO_FILTER)
  if (lastHallPulseTimeLeft != 0) {
    uint32_t interval = now - lastHallPulseTimeLeft;
    if (interval < 1000000UL) {
      if (hallPulseIntervalLeft == 0 || interval < hallPulseIntervalLeft) {
        hallPulseIntervalLeft = interval; // acelerar: reducir filtro inmediatamente
      } else {
        hallPulseIntervalLeft = (hallPulseIntervalLeft * 3UL + interval) / 4UL;
      }
      uint32_t candidate = (hallPulseIntervalLeft * OPTO_FILTER_LEFT_HALL_PERMILLE) / 1000UL;
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

/**
 * CALCULAR VELOCIDADES RPM DE LOS MOTORES
 */
void updateHallSpeeds() {
  unsigned long currentTime = millis();
  
  // Motor izquierdo
  if (currentTime - lastHallTimeLeft >= 100) {  // Actualizar cada 100ms
    // FIX Bug#3: lectura y reset atomicos — sin race condition con ISR
    noInterrupts();
    uint16_t pulsesLeft = leftHallCount;
    leftHallCount = 0;
    interrupts();

    // Calcular tiempo transcurrido
    unsigned long timeElapsed = currentTime - lastHallTimeLeft;

    // Calcular RPM: (pulsos / PPR) * (60000ms / tiempo_ms)
    if (timeElapsed > 0) {
      currentSpeedLeftHall = (float)pulsesLeft * 60000.0 / (PPR_HALL_SENSORS * timeElapsed);
    }

    lastHallTimeLeft = currentTime;
  }
  
  // Motor derecho
  if (currentTime - lastHallTimeRight >= 100) {  // Actualizar cada 100ms
    // FIX Bug#3: lectura y reset atomicos — sin race condition con ISR
    noInterrupts();
    uint16_t pulsesRight = rightHallCount;
    rightHallCount = 0;
    interrupts();

    // Calcular tiempo transcurrido
    unsigned long timeElapsed = currentTime - lastHallTimeRight;

    // Calcular RPM: (pulsos / PPR) * (60000ms / tiempo_ms)
    if (timeElapsed > 0) {
      currentSpeedRightHall = (float)pulsesRight * 60000.0 / (PPR_HALL_SENSORS * timeElapsed);
    }

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
