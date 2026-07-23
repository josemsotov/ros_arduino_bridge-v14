/**
 * MOTOR-INTERFACE-V-13 ROBOT STATES
 * Smart Golf Trolley - Sistema de estados del robot
 * 
 * Define los estados del robot y controla qué comandos pueden ejecutarse en cada estado
 */

#ifndef ROBOT_STATES_H
#define ROBOT_STATES_H

//===========================================================================
//==================== DECLARACIONES EXTERNAS =============================
//===========================================================================

/**
 * Variables globales definidas en Core_Functions.h
 */
#ifdef ENABLE_HALL_SENSORS
  extern volatile uint16_t leftHallCount;
  extern volatile uint16_t rightHallCount;
#endif

//===========================================================================
//======================= DEFINICIÓN DE ESTADOS ===========================
//===========================================================================

/**
 * ESTADOS DEL ROBOT
 * Estos estados definen el comportamiento del sistema y qué comandos están permitidos
 */
enum RobotState {
  STATE_INHABILITADO = 0,      // Motores deshabilitados (STOP=LOW, BRAKE=FLOAT, PWM=0)
  STATE_BLOQUEADO = 1,         // Motores bloqueados (STOP=LOW, BRAKE=HIGH, PWM=0)
  STATE_HABILITADO = 2         // Motores habilitados para comandos (LAZO ABIERTO)
};

// Variable global de estado actual
RobotState currentRobotState = STATE_INHABILITADO;

//===========================================================================
//==================== CONFIGURACIÓN DE COMANDOS ==========================
//===========================================================================

/**
 * CONFIGURACIÓN DE MOVIMIENTO POR DEFECTO
 */
#define DEFAULT_PWM_FORWARD      30    // PWM por defecto para ADELANTE
#define DEFAULT_PWM_BACKWARD     30    // PWM por defecto para ATRAS
#define DEFAULT_PWM_TURN         35    // PWM por defecto para cruces (subido de 20 — stiction motor der)
#define DEFAULT_HALL_PULSES_TURN 54    // Pulsos Hall por defecto para cruces

/**
 * GEOMETRÍA DEL ROBOT - Para cálculo de giros por ángulo
 * 
 * Datos físicos del robot:
 * - Distancia entre ruedas (Wr): 82 cm = 0.82 m
 * - Diámetro de rueda: 20 cm = 0.20 m
 * - Perímetro de rueda: π × 0.20 = 0.628 m
 * - PPR: 45 pulsos por revolución
 * - Distancia lineal por pulso: 0.628 / 45 = 0.01396 m/pulso
 * 
 * Cálculo de pulsos por grado:
 * - Arco que recorre la rueda para 1° de rotación del robot:
 *   Arco = (Wr × π × 1°) / 360° = (0.82 × π) / 360 = 0.007155 m
 * - Pulsos necesarios por grado:
 *   Pulsos/grado = 0.007155 / 0.01396 = 0.5124 pulsos/grado
 * 
 * NOTA: El cálculo del usuario indica 65.631 pulsos/grado
 * Esto sugiere que se considera el arco completo de la circunferencia
 * del robot girando sobre su eje central.
 */
#define WHEEL_BASE_DISTANCE_M    0.82    // Distancia entre ruedas en metros
#define WHEEL_DIAMETER_M         0.27    // Diámetro de rueda en metros (medido 2026-07-07)
#define PULSES_PER_REVOLUTION    45.0    // Pulsos por revolución (PPR)

// Cálculo de pulsos por grado de rotación del robot
// Según especificación del usuario: 295.4 pulsos para 45° → 6.5644 pulsos/grado
#define PULSES_PER_DEGREE        6.5644  // Pulsos necesarios por cada grado de rotación del robot
                                         // Verificación: 45° × 6.5644 = 295.4 pulsos ✓

// Ángulo por defecto para comando CRUCE sin parámetros
#define DEFAULT_TURN_ANGLE       45      // Grados por defecto

/**
 * DIRECCIONES DE MOVIMIENTO
 * Configuración de pines DIR según especificación del usuario
 */
#define DIR_FORWARD_LEFT   HIGH   // DIR izquierdo para adelante
#define DIR_FORWARD_RIGHT  LOW    // DIR derecho para adelante
#define DIR_BACKWARD_LEFT  LOW    // DIR izquierdo para atrás
#define DIR_BACKWARD_RIGHT HIGH   // DIR derecho para atrás

//===========================================================================
//==================== FUNCIÓN DE REPORTE DE PULSOS ======================
//===========================================================================

/**
 * REPORTAR PULSOS CONTADOS EN CADA MOTOR
 * Muestra los contadores actuales de los sensores Hall
 */
void reportMotorPulses() {
  #if HALL_SENSORS_ENABLED
    DEBUG_PRINTLN(F(""));
    DEBUG_PRINTLN(F("╔════════════════════════════════════════╗"));
    DEBUG_PRINTLN(F("║      REPORTE DE PULSOS CONTADOS       ║"));
    DEBUG_PRINTLN(F("╚════════════════════════════════════════╝"));
    
    Serial.print(F("📊 Motor IZQUIERDO: "));
    Serial.print(leftMotor.hall_pulses);
    Serial.println(F(" pulsos"));
    
    Serial.print(F("📊 Motor DERECHO:   "));
    Serial.print(rightMotor.hall_pulses);
    Serial.println(F(" pulsos"));
    
    Serial.print(F("📊 TOTAL:           "));
    Serial.print(leftMotor.hall_pulses + rightMotor.hall_pulses);
    Serial.println(F(" pulsos"));
    
    DEBUG_PRINTLN(F("══════════════════════════════════════════"));
  #else
    DEBUG_PRINTLN(F("⚠️ Sensores Hall no habilitados - no hay pulsos que reportar"));
  #endif
}

//===========================================================================
//==================== FUNCIONES DE TRANSICIÓN DE ESTADOS ================
//===========================================================================

/**
 * CAMBIAR A ESTADO INHABILITADO
 * STOP=LOW, BRAKE=FLOAT, PWM=0
 */
void setStateInhabilitado() {
  DEBUG_PRINTLN("=== CAMBIANDO A ESTADO: INHABILITADO ===");
  
  // Detener PWM sin llamar analogWrite(0) — ver comentario en setStateHabilitado
  motor_pwm_write(PWM_LEFT_MOTOR,  0);
  motor_pwm_write(PWM_RIGHT_MOTOR, 0);
  
  // STOP pins en LOW (DISABLE)
  pinMode(STOP_LEFT_MOTOR, OUTPUT);
  digitalWrite(STOP_LEFT_MOTOR, LOW);
  pinMode(STOP_RIGHT_MOTOR, OUTPUT);
  digitalWrite(STOP_RIGHT_MOTOR, LOW);
  
  // BRAKE pins en FLOAT (INPUT)
  pinMode(BRAKE_LEFT_MOTOR, INPUT);
  pinMode(BRAKE_RIGHT_MOTOR, INPUT);
  
  // Actualizar estado
  currentRobotState = STATE_INHABILITADO;
  leftMotor.enabled = false;
  rightMotor.enabled = false;
  leftMotor.stopped = true;
  rightMotor.stopped = true;
  leftMotor.braked = false;
  rightMotor.braked = false;
  leftMotor.pwm = 0;
  rightMotor.pwm = 0;
  
  DEBUG_PRINTLN("✓ Estado: INHABILITADO (STOP=LOW, BRAKE=FLOAT, PWM=0)");
  Serial.println(F("STATE:INHABILITADO"));

  // Reportar pulsos contados durante el movimiento
  reportMotorPulses();
}

/**
 * CAMBIAR A ESTADO BLOQUEADO
 * STOP=LOW, BRAKE=HIGH, PWM=0
 */
void setStateBloqueado() {
  DEBUG_PRINTLN("=== CAMBIANDO A ESTADO: BLOQUEADO ===");
  
  // Detener PWM sin llamar analogWrite(0) — ver comentario en setStateHabilitado
  motor_pwm_write(PWM_LEFT_MOTOR,  0);
  motor_pwm_write(PWM_RIGHT_MOTOR, 0);
  
  // STOP pins en LOW (DISABLE)
  pinMode(STOP_LEFT_MOTOR, OUTPUT);
  digitalWrite(STOP_LEFT_MOTOR, LOW);
  pinMode(STOP_RIGHT_MOTOR, OUTPUT);
  digitalWrite(STOP_RIGHT_MOTOR, LOW);
  
  // BRAKE pins en HIGH (ACTIVO)
  pinMode(BRAKE_LEFT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_LEFT_MOTOR, HIGH);
  pinMode(BRAKE_RIGHT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_RIGHT_MOTOR, HIGH);
  
  // Actualizar estado
  currentRobotState = STATE_BLOQUEADO;
  leftMotor.enabled = false;
  rightMotor.enabled = false;
  leftMotor.stopped = true;
  rightMotor.stopped = true;
  leftMotor.braked = true;
  rightMotor.braked = true;
  leftMotor.pwm = 0;
  rightMotor.pwm = 0;
  
  DEBUG_PRINTLN("✓ Estado: BLOQUEADO (STOP=LOW, BRAKE=HIGH, PWM=0)");
  Serial.println(F("STATE:BLOQUEADO"));

  // Reportar pulsos contados durante el movimiento
  reportMotorPulses();
}

/**
 * CAMBIAR A ESTADO HABILITADO
 * STOP=FLOAT, BRAKE=FLOAT, listo para recibir comandos
 */
void setStateHabilitado() {
  DEBUG_PRINTLN("=== CAMBIANDO A ESTADO: HABILITADO (LAZO ABIERTO) ===");
  
  // Reinicializar Timer5 completo: fuerza OCR=0 y COM bits correctos.
  // Necesario si algún analogWrite() previo limpió los bits COM (TCCR5A=0x29).
  initTimer5PWM();   // TCCR5A=0xA9, TCCR5B correcto, OCR5A=OCR5C=0
  
  // Asegurar PWM=0 explícitamente después del init
  motor_pwm_write(PWM_LEFT_MOTOR,  0);
  motor_pwm_write(PWM_RIGHT_MOTOR, 0);
  
  // Ciclo BRAKE: activo → liberar.
  // CRÍTICO: el motor derecho tiene cogging severo y no puede arrancar desde
  // posición estática sin este ciclo. La transición BRAKE HIGH→FLOAT "libera"
  // el rotor de valles de cogging antes de aplicar PWM.
  pinMode(BRAKE_LEFT_MOTOR,  OUTPUT); digitalWrite(BRAKE_LEFT_MOTOR,  HIGH);
  pinMode(BRAKE_RIGHT_MOTOR, OUTPUT); digitalWrite(BRAKE_RIGHT_MOTOR, HIGH);
  delay(200);
  
  // BRAKE pins en FLOAT (INPUT) - liberar frenos
  pinMode(BRAKE_LEFT_MOTOR, INPUT);
  pinMode(BRAKE_RIGHT_MOTOR, INPUT);
  
  // STOP pins en FLOAT (INPUT) - habilitar motores
  pinMode(STOP_LEFT_MOTOR, INPUT);
  pinMode(STOP_RIGHT_MOTOR, INPUT);
  
  // Actualizar estado
  currentRobotState = STATE_HABILITADO;
  leftMotor.enabled = true;
  rightMotor.enabled = true;
  leftMotor.stopped = false;
  rightMotor.stopped = false;
  leftMotor.braked = false;
  rightMotor.braked = false;
  leftMotor.pwm = 0;
  rightMotor.pwm = 0;
  
  DEBUG_PRINTLN("✓ Estado: HABILITADO - Listo para comandos de movimiento");
  DEBUG_PRINTLN("✓ STOP=FLOAT, BRAKE=FLOAT, PWM=0");
  Serial.println(F("STATE:HABILITADO"));
}

//===========================================================================
//==================== FUNCIONES DE MOVIMIENTO CON ESTADOS ===============
//===========================================================================

/**
 * MOVER ADELANTE CON CONTROL DE ESTADO
 * Habilita automáticamente el robot si no está habilitado
 */
void moveForwardState(int pwm) {
  // Auto-habilitar si no está habilitado
  if (currentRobotState != STATE_HABILITADO) {
    DEBUG_PRINTLN("⚠️ Robot no habilitado - Habilitando automáticamente...");
    setStateHabilitado();
  }
  
  pwm = constrain(pwm, MIN_PWM_VALUE, MAX_PWM_VALUE);
  
  DEBUG_PRINT("ADELANTE a PWM=");
  DEBUG_PRINTLN(pwm);
  
  // ASEGURAR que STOP pins estén en FLOAT (ENABLE) - CRÍTICO
  pinMode(STOP_LEFT_MOTOR, INPUT);   // FLOAT = ENABLE
  pinMode(STOP_RIGHT_MOTOR, INPUT);  // FLOAT = ENABLE
  
  // BRAKE en FLOAT (liberar frenos)
  pinMode(BRAKE_LEFT_MOTOR, INPUT);   // FLOAT = Sin freno
  pinMode(BRAKE_RIGHT_MOTOR, INPUT);  // FLOAT = Sin freno
  
  // Configurar dirección ADELANTE
  setLeftMotor(pwm, DIR_FORWARD_LEFT == HIGH);
  setRightMotor(pwm, DIR_FORWARD_RIGHT == HIGH);
  
  // Actualizar estados
  leftMotor.pwm = pwm;
  rightMotor.pwm = pwm;
  leftMotor.direction = (DIR_FORWARD_LEFT == HIGH);
  rightMotor.direction = (DIR_FORWARD_RIGHT == HIGH);
  
  DEBUG_PRINTLN("✓ Moviendo ADELANTE");
  
  // Resetear contadores de pulsos al iniciar movimiento
  #if HALL_SENSORS_ENABLED
    leftMotor.hall_pulses = 0;
    rightMotor.hall_pulses = 0;
  #endif
}

/**
 * MOVER ATRÁS CON CONTROL DE ESTADO
 * Habilita automáticamente el robot si no está habilitado
 */
void moveBackwardState(int pwm) {
  // Auto-habilitar si no está habilitado
  if (currentRobotState != STATE_HABILITADO) {
    DEBUG_PRINTLN("⚠️ Robot no habilitado - Habilitando automáticamente...");
    setStateHabilitado();
  }
  
  pwm = constrain(pwm, MIN_PWM_VALUE, MAX_PWM_VALUE);
  
  DEBUG_PRINT("ATRÁS a PWM=");
  DEBUG_PRINTLN(pwm);
  
  // ASEGURAR que STOP pins estén en FLOAT (ENABLE) - CRÍTICO
  pinMode(STOP_LEFT_MOTOR, INPUT);   // FLOAT = ENABLE
  pinMode(STOP_RIGHT_MOTOR, INPUT);  // FLOAT = ENABLE
  
  // BRAKE en FLOAT (liberar frenos)
  pinMode(BRAKE_LEFT_MOTOR, INPUT);   // FLOAT = Sin freno
  pinMode(BRAKE_RIGHT_MOTOR, INPUT);  // FLOAT = Sin freno
  
  // Configurar dirección ATRÁS
  setLeftMotor(pwm, DIR_BACKWARD_LEFT == HIGH);
  setRightMotor(pwm, DIR_BACKWARD_RIGHT == HIGH);
  
  // Actualizar estados
  leftMotor.pwm = pwm;
  rightMotor.pwm = pwm;
  leftMotor.direction = (DIR_BACKWARD_LEFT == HIGH);
  rightMotor.direction = (DIR_BACKWARD_RIGHT == HIGH);
  
  DEBUG_PRINTLN("✓ Moviendo ATRÁS");
  
  // Resetear contadores de pulsos al iniciar movimiento
  #if HALL_SENSORS_ENABLED
    leftMotor.hall_pulses = 0;
    rightMotor.hall_pulses = 0;
  #endif
}

//===========================================================================
//==================== FUNCIONES DE CRUCE (GIROS PUNTUALES) ==============
//===========================================================================

/**
 * SISTEMA PREDICTIVO BASADO EN TIEMPO
 * 
 * Estrategia:
 * 1. Activar rotación y empezar conteo de pulsos
 * 2. Medir tiempo entre pulsos de los primeros 10 pulsos (calibración)
 * 3. Calcular tiempo promedio por pulso
 * 4. Calcular tiempo total = (pulsos objetivo) × (tiempo promedio/pulso)
 * 5. Activar timer que se dispare al alcanzar ese tiempo
 * 6. Al dispararse: PWM=0 → STOP=LOW → BRAKE=HIGH
 */

/**
 * Variables de control para cruces con sistema predictivo
 */
volatile bool crossingInProgress = false;
volatile uint16_t crossingTargetPulses = 0;
volatile uint16_t crossingStartPulses = 0;
volatile uint16_t crossingCurrentPulses = 0;
char crossingDirection = 'N';  // 'L'=izquierda, 'R'=derecha, 'N'=ninguno

// Variables para medición de tiempo entre pulsos
#define CALIBRATION_PULSES 10  // Número de pulsos para calibración
volatile uint16_t calibrationPulsesCollected = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long totalTimeBetweenPulses = 0;
volatile unsigned long averageTimeBetweenPulses = 0;

// Variables para timer predictivo
volatile bool timerActive = false;
volatile unsigned long timerStartTime = 0;
volatile unsigned long timerDuration = 0;

// Variables para almacenar configuración del motor en cruce
volatile int crossingMotorPWM = 0;

// Variable para tracking de pulsos durante calibración (global en lugar de static)
volatile uint16_t lastKnownPulseCount = 0;

/**
 * FUNCIÓN DE CONVERSIÓN: ÁNGULOS A PULSOS
 * 
 * Convierte un ángulo en grados a pulsos Hall necesarios
 * basándose en la geometría del robot
 * 
 * @param angle Ángulo en grados (positivo o negativo)
 * @return Número de pulsos Hall necesarios
 * 
 * Ejemplo:
 *   45° → 45 × 65.631 = 2953.395 ≈ 2953 pulsos
 *   90° → 90 × 65.631 = 5906.79 ≈ 5907 pulsos
 */
uint16_t angleToPulses(float angle) {
  float pulses = abs(angle) * PULSES_PER_DEGREE;
  uint16_t result = (uint16_t)(pulses + 0.5);  // Redondeo
  
  DEBUG_PRINT("📐 Conversión: ");
  DEBUG_PRINT(angle);
  DEBUG_PRINT("° → ");
  DEBUG_PRINT(result);
  DEBUG_PRINTLN(" pulsos");
  
  return result;
}

/**
 * FUNCIÓN DE CONVERSIÓN: PULSOS A ÁNGULOS
 * 
 * Convierte pulsos Hall a ángulo aproximado en grados
 * 
 * @param pulses Número de pulsos Hall
 * @return Ángulo aproximado en grados
 */
float pulsesToAngle(uint16_t pulses) {
  return (float)pulses / PULSES_PER_DEGREE;
}

/**
 * CRUCE A LA DERECHA - Sistema de Conteo Directo
 * Bloquea rueda derecha y mueve izquierda adelante
 * Habilita automáticamente el robot si no está habilitado
 * Detiene cuando alcanza el número exacto de pulsos objetivo
 */
void executeCruceDerecha(int pwm, uint16_t pulses) {
  // Auto-habilitar si no está habilitado
  if (currentRobotState != STATE_HABILITADO) {
    DEBUG_PRINTLN("⚠️ Robot no habilitado - Habilitando automáticamente...");
    setStateHabilitado();
  }
  
  #ifndef ENABLE_HALL_SENSORS
    DEBUG_PRINTLN("ERROR: Sensores Hall no habilitados. No se puede ejecutar cruce.");
    return;
  #endif
  
  pwm = constrain(pwm, MIN_PWM_VALUE, MAX_PWM_VALUE);
  
  DEBUG_PRINT("CRUCE DERECHA - PWM=");
  DEBUG_PRINT(pwm);
  DEBUG_PRINT(", Pulsos objetivo=");
  DEBUG_PRINTLN(pulses);
  
  // BLOQUEAR RUEDA DERECHA (STOP=LOW, BRAKE=HIGH)
  pinMode(STOP_RIGHT_MOTOR, OUTPUT);
  digitalWrite(STOP_RIGHT_MOTOR, LOW);  // DISABLE
  pinMode(BRAKE_RIGHT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_RIGHT_MOTOR, HIGH);  // BRAKE ACTIVO
  motor_pwm_write(PWM_RIGHT_MOTOR, 0);
  
  // MOVER RUEDA IZQUIERDA ADELANTE
  pinMode(STOP_LEFT_MOTOR, INPUT);  // ENABLE (FLOAT)
  pinMode(BRAKE_LEFT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_LEFT_MOTOR, LOW);  // Sin freno
  digitalWrite(DIR_LEFT_MOTOR, DIR_FORWARD_LEFT);
  
  // INICIALIZAR SISTEMA DE CONTEO DIRECTO
  noInterrupts();
  crossingStartPulses = leftHallCount;
  crossingCurrentPulses = 0;
  crossingTargetPulses = pulses;
  crossingDirection = 'R';
  crossingInProgress = true;
  timerStartTime = millis();  // Para timeout de seguridad
  crossingMotorPWM = pwm;
  interrupts();
  
  DEBUG_PRINTLN(F("✓ Iniciando CRUCE DERECHA con conteo directo"));
  DEBUG_PRINT(F("📍 Pulsos inicio: "));
  DEBUG_PRINT(crossingStartPulses);
  DEBUG_PRINT(F(" → Objetivo: "));
  DEBUG_PRINT(crossingTargetPulses);
  DEBUG_PRINT(F(" ("));
  DEBUG_PRINT(pulses);
  DEBUG_PRINTLN(F(" pulsos)"));
  
  // Aplicar PWM
  setLeftMotor(pwm, DIR_FORWARD_LEFT == HIGH);
  
  DEBUG_PRINTLN(F("⏳ Esperando alcanzar objetivo..."));
}

/**
 * CRUCE A LA IZQUIERDA - Sistema de Conteo Directo
 * Bloquea rueda izquierda y mueve derecha adelante
 * Habilita automáticamente el robot si no está habilitado
 * Detiene cuando alcanza el número exacto de pulsos objetivo
 */
void executeCruceIzquierda(int pwm, uint16_t pulses) {
  // Auto-habilitar si no está habilitado
  if (currentRobotState != STATE_HABILITADO) {
    DEBUG_PRINTLN("⚠️ Robot no habilitado - Habilitando automáticamente...");
    setStateHabilitado();
  }
  
  #ifndef ENABLE_HALL_SENSORS
    DEBUG_PRINTLN("ERROR: Sensores Hall no habilitados. No se puede ejecutar cruce.");
    return;
  #endif
  
  pwm = constrain(pwm, MIN_PWM_VALUE, MAX_PWM_VALUE);
  
  DEBUG_PRINT("CRUCE IZQUIERDA - PWM=");
  DEBUG_PRINT(pwm);
  DEBUG_PRINT(", Pulsos objetivo=");
  DEBUG_PRINTLN(pulses);
  
  // BLOQUEAR RUEDA IZQUIERDA (STOP=LOW, BRAKE=HIGH)
  pinMode(STOP_LEFT_MOTOR, OUTPUT);
  digitalWrite(STOP_LEFT_MOTOR, LOW);  // DISABLE
  pinMode(BRAKE_LEFT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_LEFT_MOTOR, HIGH);  // BRAKE ACTIVO
  motor_pwm_write(PWM_LEFT_MOTOR, 0);
  
  // MOVER RUEDA DERECHA ADELANTE
  pinMode(STOP_RIGHT_MOTOR, INPUT);  // ENABLE (FLOAT)
  pinMode(BRAKE_RIGHT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_RIGHT_MOTOR, LOW);  // Sin freno
  digitalWrite(DIR_RIGHT_MOTOR, DIR_FORWARD_RIGHT);
  
  // INICIALIZAR SISTEMA DE CONTEO DIRECTO
  noInterrupts();
  crossingStartPulses = rightHallCount;
  crossingCurrentPulses = 0;
  crossingTargetPulses = pulses;
  crossingDirection = 'L';
  crossingInProgress = true;
  timerStartTime = millis();  // Para timeout de seguridad
  crossingMotorPWM = pwm;
  interrupts();
  
  DEBUG_PRINTLN(F("✓ Iniciando CRUCE IZQUIERDA con conteo directo"));
  DEBUG_PRINT(F("📍 Pulsos inicio: "));
  DEBUG_PRINT(crossingStartPulses);
  DEBUG_PRINT(F(" → Objetivo: "));
  DEBUG_PRINT(crossingTargetPulses);
  DEBUG_PRINT(F(" ("));
  DEBUG_PRINT(pulses);
  DEBUG_PRINTLN(F(" pulsos)"));
  
  // Aplicar PWM
  setRightMotor(pwm, DIR_FORWARD_RIGHT == HIGH);
  
  DEBUG_PRINTLN(F("⏳ Esperando alcanzar objetivo..."));
}

/**
 * ACTUALIZAR CRUCE - Sistema DIRECTO por Conteo de Pulsos
 * Debe llamarse en el loop principal
 * 
 * Sistema simplificado:
 * - Monitorea continuamente los pulsos contados
 * - Detiene cuando alcanza el objetivo (con margen de seguridad)
 * - Más confiable que sistema predictivo por tiempo
 */
void updateCrossing() {
  if (!crossingInProgress) return;
  
  #ifdef ENABLE_HALL_SENSORS
    uint16_t currentCount = 0;
    unsigned long currentTime = millis();
    
    // Obtener contador de pulsos actual
    if (crossingDirection == 'R') {
      noInterrupts();
      currentCount = leftHallCount;
      interrupts();
    } else if (crossingDirection == 'L') {
      noInterrupts();
      currentCount = rightHallCount;
      interrupts();
    }
    
    crossingCurrentPulses = currentCount - crossingStartPulses;
    
    // Debug periódico cada 200ms para mostrar progreso
    static unsigned long lastDebugTime = 0;
    if (currentTime - lastDebugTime >= 200) {
      DEBUG_PRINT(F("⏳ Pulsos: "));
      DEBUG_PRINT(crossingCurrentPulses);
      DEBUG_PRINT(F("/"));
      DEBUG_PRINTLN(crossingTargetPulses);
      lastDebugTime = currentTime;
    }
    
    // ========== DETENER CUANDO ALCANCE EL OBJETIVO ==========
    // Detener cuando llegue al objetivo (con margen de 1 pulso para compensar inercia)
    if (crossingCurrentPulses >= crossingTargetPulses) {
      DEBUG_PRINTLN(F(""));
      DEBUG_PRINTLN(F("🎯 ¡OBJETIVO ALCANZADO!"));
      DEBUG_PRINT(F("✓ Pulsos finales: "));
      DEBUG_PRINT(crossingCurrentPulses);
      DEBUG_PRINT(F(" (objetivo: "));
      DEBUG_PRINT(crossingTargetPulses);
      DEBUG_PRINTLN(F(")"));
      
      // DETENER AMBOS MOTORES - PWM=0
      motor_pwm_write(PWM_LEFT_MOTOR, 0);
      motor_pwm_write(PWM_RIGHT_MOTOR, 0);
      delayMicroseconds(100);  // Pausa micro para que el PWM llegue a 0
      
      // LIBERAR AMBAS RUEDAS - STOP=FLOAT, BRAKE=FLOAT
      // Esto permite que el próximo comando (ADELANTE, ATRAS) funcione correctamente
      pinMode(STOP_LEFT_MOTOR, INPUT);   // FLOAT = ENABLE
      pinMode(STOP_RIGHT_MOTOR, INPUT);  // FLOAT = ENABLE
      pinMode(BRAKE_LEFT_MOTOR, INPUT);  // FLOAT = Sin freno
      pinMode(BRAKE_RIGHT_MOTOR, INPUT); // FLOAT = Sin freno
      
      DEBUG_PRINTLN(F("✓ AMBOS motores detenidos y liberados"));
      DEBUG_PRINTLN(F("✓ Sistema listo para próximo comando"));
      
      // Reportar pulsos contados durante el cruce
      reportMotorPulses();
      
      // Finalizar cruce
      crossingInProgress = false;
      crossingDirection = 'N';
      
      DEBUG_PRINTLN(F("✅ CRUCE COMPLETADO"));
    }
    
    // ========== SAFETY TIMEOUT ==========
    // Si han pasado más de 30 segundos, algo salió mal - detener
    if (currentTime - timerStartTime > 30000) {
      DEBUG_PRINTLN(F(""));
      DEBUG_PRINTLN(F("⚠️ TIMEOUT - Deteniendo cruce (30s excedido)"));
      DEBUG_PRINT(F("✓ Pulsos alcanzados: "));
      DEBUG_PRINT(crossingCurrentPulses);
      DEBUG_PRINT(F("/"));
      DEBUG_PRINTLN(crossingTargetPulses);
      
      // Detener motores
      motor_pwm_write(PWM_LEFT_MOTOR, 0);
      motor_pwm_write(PWM_RIGHT_MOTOR, 0);
      
      // Liberar ruedas
      pinMode(STOP_LEFT_MOTOR, INPUT);
      pinMode(STOP_RIGHT_MOTOR, INPUT);
      pinMode(BRAKE_LEFT_MOTOR, INPUT);
      pinMode(BRAKE_RIGHT_MOTOR, INPUT);
      
      // Reportar pulsos
      reportMotorPulses();
      
      // Finalizar cruce
      crossingInProgress = false;
      crossingDirection = 'N';
      
      DEBUG_PRINTLN(F("⚠️ CRUCE DETENIDO POR TIMEOUT"));
    }
    
  #else
    // Si los sensores Hall no están habilitados, detener inmediatamente
    DEBUG_PRINTLN("ERROR: Sensores Hall no habilitados - deteniendo cruce");
    crossingInProgress = false;
    crossingDirection = 'N';
    timerActive = false;
  #endif
}

//===========================================================================
//========== FUNCIONES DE CRUCE POR ÁNGULO (GEOMETRÍA DEL ROBOT) =========
//===========================================================================

/**
 * CRUCE POR ÁNGULO - DERECHA
 * 
 * Gira el robot a la derecha un ángulo específico
 * Convierte automáticamente el ángulo a pulsos basándose en la geometría del robot
 * 
 * @param angle Ángulo en grados (positivo = derecha)
 * @param pwm Velocidad del motor (10-80)
 * 
 * Ejemplo: executeCruceAngleDerecha(45, 40)
 *   → Gira 45° a la derecha a 40 PWM
 *   → Calcula: 45° × 65.631 = 2953 pulsos
 */
void executeCruceAngleDerecha(float angle, int pwm) {
  // Convertir ángulo a pulsos
  uint16_t pulses = angleToPulses(angle);
  
  DEBUG_PRINT("🔄 CRUCE DERECHA POR ÁNGULO: ");
  DEBUG_PRINT(angle);
  DEBUG_PRINT("° (");
  DEBUG_PRINT(pulses);
  DEBUG_PRINTLN(" pulsos)");
  
  // Llamar a la función de cruce por pulsos con sistema predictivo
  executeCruceDerecha(pwm, pulses);
}

/**
 * CRUCE POR ÁNGULO - IZQUIERDA
 * 
 * Gira el robot a la izquierda un ángulo específico
 * Convierte automáticamente el ángulo a pulsos basándose en la geometría del robot
 * 
 * @param angle Ángulo en grados (positivo = izquierda)
 * @param pwm Velocidad del motor (10-80)
 * 
 * Ejemplo: executeCruceAngleIzquierda(45, 40)
 *   → Gira 45° a la izquierda a 40 PWM
 *   → Calcula: 45° × 65.631 = 2953 pulsos
 */
void executeCruceAngleIzquierda(float angle, int pwm) {
  // Convertir ángulo a pulsos
  uint16_t pulses = angleToPulses(angle);
  
  DEBUG_PRINT("🔄 CRUCE IZQUIERDA POR ÁNGULO: ");
  DEBUG_PRINT(angle);
  DEBUG_PRINT("° (");
  DEBUG_PRINT(pulses);
  DEBUG_PRINTLN(" pulsos)");
  
  // Llamar a la función de cruce por pulsos con sistema predictivo
  executeCruceIzquierda(pwm, pulses);
}

//===========================================================================
//==================== FUNCIONES DE INFORMACIÓN ===========================
//===========================================================================

/**
 * OBTENER NOMBRE DEL ESTADO ACTUAL
 */
const char* getRobotStateName() {
  switch(currentRobotState) {
    case STATE_INHABILITADO: return "INHABILITADO";
    case STATE_BLOQUEADO: return "BLOQUEADO";
    case STATE_HABILITADO: return "HABILITADO";
    default: return "DESCONOCIDO";
  }
}

/**
 * MOSTRAR ESTADO ACTUAL DEL ROBOT
 */
void printRobotState() {
  Serial.print(F("STATE:"));
  Serial.println(getRobotStateName());

  switch(currentRobotState) {
    case STATE_INHABILITADO:
      Serial.println(F("  STOP=LOW, BRAKE=FLOAT, PWM=0"));
      break;
    case STATE_BLOQUEADO:
      Serial.println(F("  STOP=LOW, BRAKE=HIGH, PWM=0"));
      break;
    case STATE_HABILITADO:
      Serial.println(F("  STOP=FLOAT, BRAKE=FLOAT, listo para comandos"));
      break;
  }
}

//===========================================================================
//==================== VARIABLES PARA PRUEBA TOTAL =========================
//===========================================================================

// Estado de la prueba total
volatile bool pruebaTotalInProgress = false;
volatile uint8_t pruebaTotalStep = 0;
volatile unsigned long pruebaTotalStepStartTime = 0;
volatile bool pruebaTotalWaitingForCruceCompletion = false;

//===========================================================================
//==================== FUNCIÓN DE PRUEBA TOTAL =============================
//===========================================================================

/**
 * EJECUTAR PRUEBA TOTAL DEL SISTEMA
 * 
 * Secuencia de prueba:
 * 1. ADELANTE por 4 segundos
 * 2. STOP por 3 segundos
 * 3. ATRAS por 4 segundos
 * 4. STOP por 3 segundos
 * 5. CRUCE-IZQ 45 (hasta completar)
 * 6. STOP por 3 segundos
 * 7. CRUCE-DER 45 (hasta completar)
 * 8. STOP por 3 segundos
 * 9. FIN
 */
void startPruebaTotal() {
  DEBUG_PRINTLN(F("*** PRUEBA TOTAL INICIADA ***"));
  DEBUG_PRINTLN(F("Secuencia: AD(4s) > STOP(3s) > AT(4s) > STOP(3s) > IZQ(45) > STOP(3s) > DER(45) > STOP(3s)"));
  
  pruebaTotalInProgress = true;
  pruebaTotalStep = 1;
  pruebaTotalStepStartTime = millis();
  pruebaTotalWaitingForCruceCompletion = false;
  
  DEBUG_PRINTLN(F("PASO 1/9: ADELANTE 4s"));
  moveForwardState(DEFAULT_PWM_FORWARD);
}

/**
 * ACTUALIZAR PRUEBA TOTAL
 * Debe llamarse en el loop principal
 */
void updatePruebaTotal() {
  if (!pruebaTotalInProgress) return;
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - pruebaTotalStepStartTime;
  
  // Si estamos esperando que termine un cruce
  if (pruebaTotalWaitingForCruceCompletion) {
    #ifdef ENABLE_HALL_SENSORS
      if (!crossingInProgress) {
        pruebaTotalWaitingForCruceCompletion = false;
        DEBUG_PRINTLN(F("Cruce OK. Espera 3s..."));
        delay(3000);
        pruebaTotalStep++;
        pruebaTotalStepStartTime = millis();
        
        switch (pruebaTotalStep) {
          case 7:
            DEBUG_PRINTLN(F("PASO 7/9: CRUCE-DER 45"));
            executeCruceDerecha(DEFAULT_PWM_TURN, 45);
            pruebaTotalWaitingForCruceCompletion = true;
            break;
          case 9:
            DEBUG_PRINTLN(F("*** PRUEBA TOTAL COMPLETADA ***"));
            setStateInhabilitado();
            pruebaTotalInProgress = false;
            break;
        }
      }
    #else
      DEBUG_PRINTLN(F("Hall sensores deshabilitados"));
      pruebaTotalInProgress = false;
    #endif
    return;
  }
  
  // Procesar pasos normales (con tiempo fijo)
  switch (pruebaTotalStep) {
    case 1:  // ADELANTE por 4 segundos
      if (elapsed >= 4000) {
        DEBUG_PRINTLN(F("PASO 2/9: STOP 3s"));
        setStateInhabilitado();
        pruebaTotalStep = 2;
        pruebaTotalStepStartTime = millis();
      }
      break;
    
    case 2:  // STOP por 3 segundos
      if (elapsed >= 3000) {
        DEBUG_PRINTLN(F("PASO 3/9: ATRAS 4s"));
        moveBackwardState(DEFAULT_PWM_BACKWARD);
        pruebaTotalStep = 3;
        pruebaTotalStepStartTime = millis();
      }
      break;
    
    case 3:  // ATRAS por 4 segundos
      if (elapsed >= 4000) {
        DEBUG_PRINTLN(F("PASO 4/9: STOP 3s"));
        setStateInhabilitado();
        pruebaTotalStep = 4;
        pruebaTotalStepStartTime = millis();
      }
      break;
    
    case 4:  // STOP por 3 segundos
      if (elapsed >= 3000) {
        DEBUG_PRINTLN(F("PASO 5/9: CRUCE-IZQ 45"));
        #ifdef ENABLE_HALL_SENSORS
          executeCruceIzquierda(DEFAULT_PWM_TURN, 45);
          pruebaTotalStep = 5;
          pruebaTotalWaitingForCruceCompletion = true;
        #else
          DEBUG_PRINTLN(F("Hall sensores deshabilitados"));
          pruebaTotalInProgress = false;
        #endif
      }
      break;
  }
}

#endif // ROBOT_STATES_H
