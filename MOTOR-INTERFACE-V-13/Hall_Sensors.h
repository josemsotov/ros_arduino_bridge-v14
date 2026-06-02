/**
 * MOTOR-INTERFACE-V-13 HALL SENSORS
 * Smart Golf Trolley - Manejo de sensores Hall
 */

#ifndef HALL_SENSORS_H
#define HALL_SENSORS_H

#ifdef ENABLE_HALL_SENSORS

// Forward declarations — variables definidas en Core_Functions.h
// (Hall_Sensors.h se incluye ANTES de Core_Functions.h en Modules.h)
extern volatile uint16_t leftHallCount;
extern volatile uint16_t rightHallCount;
extern volatile int32_t  leftHallSigned;
extern volatile int32_t  rightHallSigned;

// Forward declaration — variable definida en Motor_Control.h (incluido antes)
extern volatile bool motorGoingForwardLeft;
extern volatile bool motorGoingForwardRight;

// Forward declarations — Odometry.h se incluye DESPUÉS de Hall_Sensors.h
#ifdef ENABLE_ODOMETRY
  extern int32_t odom_prevLeftSigned;
  extern int32_t odom_prevRightSigned;
  void odom_update();
#endif

//===========================================================================
//==================== FUNCIONES DE INTERRUPCIÓN =========================
//===========================================================================

/**
 * RUTINAS DE SERVICIO DE INTERRUPCIÓN (ISR)
 * Lee flag volatile motorGoingForwardLeft/Right (actualizado en Motor_Control.h)
 * para determinar el signo del contador — sin digitalRead dentro del ISR.
 */
void leftHallISR() {
  leftHallCount++;
  // Usa flag volatile actualizado en setLeftMotor — sin digitalRead
  if (motorGoingForwardLeft) {
    leftHallSigned++;
  } else {
    leftHallSigned--;
  }
}

void rightHallISR() {
  rightHallCount++;
  if (motorGoingForwardRight) {
    rightHallSigned++;
  } else {
    rightHallSigned--;
  }
}

//===========================================================================
//==================== VARIABLES SENSORES HALL ============================
//===========================================================================

/**
 * CONTADORES DE PULSOS HALL (definidos en Core_Functions.h)
 * volatile uint16_t leftHallCount = 0;
 * volatile uint16_t rightHallCount = 0;
 */

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
  attachInterrupt(digitalPinToInterrupt(HALL_LEFT_MOTOR), leftHallISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_RIGHT_MOTOR), rightHallISR, RISING);
  
  // Inicializar contadores
  leftHallCount = 0;
  rightHallCount = 0;
  lastHallTimeLeft = millis();
  lastHallTimeRight = millis();
  
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
    // Calcular diferencia de pulsos
    uint16_t pulsesLeft = leftHallCount;
    
    // Calcular tiempo transcurrido
    unsigned long timeElapsed = currentTime - lastHallTimeLeft;
    
    // Calcular RPM: (pulsos / PPR) * (60000ms / tiempo_ms)
    if (timeElapsed > 0) {
      currentSpeedLeftHall = (float)pulsesLeft * 60000.0 / (PPR_HALL_SENSORS * timeElapsed);
    }
    
    // Resetear para próxima medición
    leftHallCount = 0;
    lastHallTimeLeft = currentTime;
  }
  
  // Motor derecho
  if (currentTime - lastHallTimeRight >= 100) {  // Actualizar cada 100ms
    // Calcular diferencia de pulsos
    uint16_t pulsesRight = rightHallCount;
    
    // Calcular tiempo transcurrido
    unsigned long timeElapsed = currentTime - lastHallTimeRight;
    
    // Calcular RPM: (pulsos / PPR) * (60000ms / tiempo_ms)
    if (timeElapsed > 0) {
      currentSpeedRightHall = (float)pulsesRight * 60000.0 / (PPR_HALL_SENSORS * timeElapsed);
    }
    
    // Resetear para próxima medición
    rightHallCount = 0;
    lastHallTimeRight = currentTime;
  }

  // Actualizar odometría en cada ciclo Hall
  #ifdef ENABLE_ODOMETRY
    odom_update();
  #endif
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
  leftHallCount   = 0;
  rightHallCount  = 0;
  leftHallSigned  = 0;
  rightHallSigned = 0;
  interrupts();

  // Sincronizar baselines de odometría (evita delta incorrecto en próxima actualización)
  #ifdef ENABLE_ODOMETRY
    noInterrupts();
    odom_prevLeftSigned  = 0;
    odom_prevRightSigned = 0;
    interrupts();
  #endif

  lastHallTimeLeft  = millis();
  lastHallTimeRight = millis();
  currentSpeedLeftHall  = 0;
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