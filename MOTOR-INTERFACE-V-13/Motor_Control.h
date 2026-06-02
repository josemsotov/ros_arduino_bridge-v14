/**
 * MOTOR-INTERFACE-V-13 MOTOR CONTROL
 * Smart Golf Trolley - Control básico de motores
 * 
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║  ⚠️  ADVERTENCIA CRÍTICA - NO MODIFICAR LÓGICA DE MOTORES  ⚠️        ║
 * ╠═══════════════════════════════════════════════════════════════════════╣
 * ║                                                                       ║
 * ║  Las funciones setLeftMotor() y setRightMotor() están CALIBRADAS    ║
 * ║  y VERIFICADAS para funcionar correctamente en LAZO ABIERTO.        ║
 * ║                                                                       ║
 * ║  ❌ NO CAMBIAR:                                                       ║
 * ║     - Lógica de dirección (HIGH/LOW en pines DIR)                   ║
 * ║     - Lógica de STOP (FLOAT=enable, LOW=disable)                    ║
 * ║     - Lógica de BRAKE (HIGH=brake, LOW=release)                     ║
 * ║     - Mapeo de pines                                                 ║
 * ║                                                                       ║
 * ║  ✅ Si hay problemas de dirección en control PID:                    ║
 * ║     - Corregir en PID_Control.h (cinemática diferencial)            ║
 * ║     - NO modificar Motor_Control.h                                   ║
 * ║                                                                       ║
 * ║  📅 Verificado funcionando correctamente: 2025-10-12                ║
 * ║                                                                       ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

//===========================================================================
//==================== DECLARACIONES FORWARD ==============================
//===========================================================================

// Función definida en Core_Functions.h
void emergencyStop();

//===========================================================================
//======================= VARIABLES DE CONTROL DE MOTORES =================
//===========================================================================

/**
 * ESTADOS DE LOS MOTORES
 */
struct MotorState {
  int pwm;              // Valor PWM actual (0-255)
  bool direction;       // Dirección: true=adelante, false=atrás
  bool stopped;         // Estado STOP
  bool braked;          // Estado BRAKE
  bool enabled;         // Motor habilitado
};

MotorState leftMotor = {0, true, true, true, false};
MotorState rightMotor = {0, true, true, true, false};

/**
 * LÍMITES DE SEGURIDAD
 */
#define MIN_PWM_VALUE     10    // PWM mínimo para evitar stall
#define MAX_PWM_VALUE     80   // PWM máximo permitido

#define EMERGENCY_STOP_TIME  300000  // Tiempo máximo sin comando (ms) - 5 minutos

unsigned long lastCommandTime = 0;
bool safetyCheckEnabled = true;  // Variable para habilitar/deshabilitar safety check

// Variables de dirección configurables en runtime via DIR_IZQ / DIR_DER
// Guardadas en EEPROM addr 0x10 (IZQ), 0x11 (DER), 0x12 (firma 0xAC)
// VERIFICADO 2026-04-24: IZQ=HIGH, DER=LOW (tras swap de pines)
bool dirForwardLeft  = HIGH;  // Nivel lógico del pin DIR izquierdo para ADELANTE (verificado)
bool dirForwardRight = LOW;   // Nivel lógico del pin DIR derecho  para ADELANTE (verificado)

// Flags volatile para las ISR de Hall — true=adelante, false=atrás
// Se actualizan en setLeftMotor/setRightMotor al mismo tiempo que el pin DIR
volatile bool motorGoingForwardLeft  = true;
volatile bool motorGoingForwardRight = true;

//===========================================================================
//======================= FUNCIONES DE INICIALIZACIÓN ======================
//===========================================================================

/**
 * INICIALIZAR PINES DE MOTORES
 */
void initializeMotorPins() {
  // Configurar pines de control de motores como salidas
  pinMode(PWM_LEFT_MOTOR, OUTPUT);
  pinMode(DIR_LEFT_MOTOR, OUTPUT);
  pinMode(BRAKE_LEFT_MOTOR, OUTPUT);
  
  pinMode(PWM_RIGHT_MOTOR, OUTPUT);
  pinMode(DIR_RIGHT_MOTOR, OUTPUT);
  pinMode(BRAKE_RIGHT_MOTOR, OUTPUT);
  
  // *** ESTADO INICIAL: MOTORES INHABILITADOS (ESTÁTICO) ***
  // FORZAR STOP pins en LOW = DISABLE/MOTORES INHABILITADOS
  pinMode(STOP_LEFT_MOTOR, OUTPUT);
  digitalWrite(STOP_LEFT_MOTOR, LOW);   // LOW = DISABLE
  delay(10);  // Pequeña pausa para asegurar el estado
  pinMode(STOP_RIGHT_MOTOR, OUTPUT);
  digitalWrite(STOP_RIGHT_MOTOR, LOW);  // LOW = DISABLE
  delay(10);  // Pequeña pausa para asegurar el estado
  
  // VERIFICAR ESTADO (forzar LOW otra vez para asegurar)
  digitalWrite(STOP_LEFT_MOTOR, LOW);   // FORZAR LOW
  digitalWrite(STOP_RIGHT_MOTOR, LOW);  // FORZAR LOW
  
  // Estado inicial: frenos activados por seguridad
  pinMode(BRAKE_LEFT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_LEFT_MOTOR, HIGH);  // HIGH = BRAKE ACTIVO
  pinMode(BRAKE_RIGHT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_RIGHT_MOTOR, HIGH); // HIGH = BRAKE ACTIVO
  analogWrite(PWM_LEFT_MOTOR, 0);
  analogWrite(PWM_RIGHT_MOTOR, 0);
  
  // Inicializar estados como INHABILITADOS
  leftMotor.enabled = false;   // MOTORES INHABILITADOS al inicio
  rightMotor.enabled = false;  // MOTORES INHABILITADOS al inicio
  leftMotor.stopped = true;    // STOP activo
  rightMotor.stopped = true;   // STOP activo
  leftMotor.braked = true;     // BRAKE activo
  rightMotor.braked = true;    // BRAKE activo
  
  DEBUG_PRINTLN("Pines de motores inicializados - ESTADO ESTÁTICO (INHABILITADOS)");
}

/**
 * FUNCIÓN PARA FORZAR STOP PINS A LOW EN INICIALIZACIÓN
 * Se ejecuta después de la inicialización para garantizar el estado
 */
void forceStopPinsLowInit() {
  pinMode(STOP_LEFT_MOTOR, OUTPUT);
  pinMode(STOP_RIGHT_MOTOR, OUTPUT);
  
  // Forzar múltiples veces para asegurar el estado
  for(int i = 0; i < 3; i++) {
    digitalWrite(STOP_LEFT_MOTOR, LOW);
    digitalWrite(STOP_RIGHT_MOTOR, LOW);
    delay(5);
  }
  
  // Actualizar estados
  leftMotor.enabled = false;
  rightMotor.enabled = false;
  leftMotor.stopped = true;
  rightMotor.stopped = true;
  
  DEBUG_PRINTLN("*** STOP PINS FORZADOS A LOW - MOTORES INHABILITADOS ***");
}

/**
 * INICIALIZAR OPTOENCODERS (DUMMY)
 * Esta función existe para evitar errores si ENABLE_OPTO_ENCODERS no está definido
 */
void initializeOptoEncoders() {
  #ifdef ENABLE_OPTO_ENCODERS
    // Configurar pines de OptoEncoders
    pinMode(OPTO_LEFT_MOTOR, INPUT_PULLUP);
    pinMode(OPTO_RIGHT_MOTOR, INPUT_PULLUP);
    
    // Configurar interrupciones (las funciones ISR están definidas más abajo)
    // attachInterrupt(digitalPinToInterrupt(OPTO_LEFT_MOTOR), leftOptoISR, RISING);
    // attachInterrupt(digitalPinToInterrupt(OPTO_RIGHT_MOTOR), rightOptoISR, RISING);
    // Nota: Interrupciones de OptoEncoders deshabilitadas temporalmente
    
    DEBUG_PRINTLN("OptoEncoders inicializados");
  #else
    DEBUG_PRINTLN("OptoEncoders deshabilitados");
  #endif
}

/**
 * FUNCIONES ISR PARA OPTOENCODERS
 * Estas funciones se definen aquí para evitar errores si se referencian
 */
#ifdef ENABLE_OPTO_ENCODERS
// Declaraciones externas para variables de OptoEncoders
extern volatile uint16_t leftOptoCount;
extern volatile uint16_t rightOptoCount;

void leftOptoISR() {
  leftOptoCount++;
}

void rightOptoISR() {
  rightOptoCount++;
}
#endif

//===========================================================================
//======================= FUNCIONES BÁSICAS DE MOTORES ====================
//===========================================================================

/**
 * ╔═══════════════════════════════════════════════════════════════════════╗
 * ║  ⚠️  SECCIÓN CRÍTICA - NO MODIFICAR  ⚠️                               ║
 * ╠═══════════════════════════════════════════════════════════════════════╣
 * ║                                                                       ║
 * ║  Las funciones setLeftMotor() y setRightMotor() a continuación       ║
 * ║  están VERIFICADAS y funcionan correctamente.                        ║
 * ║                                                                       ║
 * ║  ❌ NO MODIFICAR LA LÓGICA DE:                                        ║
 * ║     • Dirección (digitalWrite DIR pin)                               ║
 * ║     • PWM (analogWrite PWM pin)                                      ║
 * ║     • STOP (pinMode/digitalWrite STOP pin)                           ║
 * ║     • BRAKE (digitalWrite BRAKE pin)                                 ║
 * ║                                                                       ║
 * ║  ✅ Si un motor va en dirección incorrecta en control PID:           ║
 * ║     → Corregir cinemática diferencial en PID_Control.h               ║
 * ║     → NO tocar Motor_Control.h                                       ║
 * ║                                                                       ║
 * ║  📝 Lógica actual (CORRECTA y VERIFICADA):                           ║
 * ║     • direction=true  → HIGH en DIR → Motor adelante                 ║
 * ║     • direction=false → LOW en DIR  → Motor atrás                    ║
 * ║     • STOP=FLOAT (INPUT) → Motor habilitado                          ║
 * ║     • STOP=LOW (OUTPUT) → Motor deshabilitado                        ║
 * ║     • BRAKE=HIGH → Freno activado                                    ║
 * ║     • BRAKE=LOW → Freno liberado                                     ║
 * ║                                                                       ║
 * ║  📅 Última verificación: 2025-10-12                                  ║
 * ║  ✅ Estado: FUNCIONANDO CORRECTAMENTE                                ║
 * ║                                                                       ║
 * ╚═══════════════════════════════════════════════════════════════════════╝
 */

/**
 * CONFIGURAR MOTOR IZQUIERDO
 */
void setLeftMotor(int pwm, bool direction) {
  // Validar PWM
  pwm = constrain(pwm, 0, MAX_PWM_VALUE);
  
  // Si PWM es muy bajo, detener motor
  if (pwm > 0 && pwm < MIN_PWM_VALUE) {
    pwm = MIN_PWM_VALUE;
  }
  
  // Actualizar estado
  leftMotor.pwm = pwm;
  leftMotor.direction = direction;
  leftMotor.stopped = (pwm == 0);
  
  // Aplicar configuración al hardware
  if (!leftMotor.braked && leftMotor.enabled) {
    // direction=true → adelante (usa dirForwardLeft para mapeo físico)
    // direction=false → atrás
    bool pinVal = direction ? dirForwardLeft : !dirForwardLeft;
    digitalWrite(DIR_LEFT_MOTOR, pinVal);
    motorGoingForwardLeft = direction;   // ← actualizar flag para ISR Hall
    analogWrite(PWM_LEFT_MOTOR, pwm);
    // STOP pin: FLOAT = ENABLE, LOW = DISABLE
    if (leftMotor.stopped) {
      pinMode(STOP_LEFT_MOTOR, OUTPUT);
      digitalWrite(STOP_LEFT_MOTOR, LOW);  // LOW = DISABLE
    } else {
      pinMode(STOP_LEFT_MOTOR, INPUT);     // FLOAT = ENABLE
    }
  }
  
  lastCommandTime = millis();
  
  DEBUG_PRINT("Motor IZQ: PWM=");
  DEBUG_PRINT(pwm);
  DEBUG_PRINT(" DIR=");
  DEBUG_PRINTLN(direction ? "FWD" : "BWD");
}

/**
 * CONFIGURAR MOTOR DERECHO
 */
void setRightMotor(int pwm, bool direction) {
  // Validar PWM
  pwm = constrain(pwm, 0, MAX_PWM_VALUE);
  
  // Si PWM es muy bajo, detener motor
  if (pwm > 0 && pwm < MIN_PWM_VALUE) {
    pwm = MIN_PWM_VALUE;
  }
  
  // Actualizar estado
  rightMotor.pwm = pwm;
  rightMotor.direction = direction;
  rightMotor.stopped = (pwm == 0);
  
  // Aplicar configuración al hardware
  if (!rightMotor.braked && rightMotor.enabled) {
    // direction=true → adelante (usa dirForwardRight para mapeo físico)
    // direction=false → atrás
    bool pinVal = direction ? dirForwardRight : !dirForwardRight;
    digitalWrite(DIR_RIGHT_MOTOR, pinVal);
    motorGoingForwardRight = direction;  // ← actualizar flag para ISR Hall
    analogWrite(PWM_RIGHT_MOTOR, pwm);
    // STOP pin: FLOAT = ENABLE, LOW = DISABLE
    if (rightMotor.stopped) {
      pinMode(STOP_RIGHT_MOTOR, OUTPUT);
      digitalWrite(STOP_RIGHT_MOTOR, LOW);  // LOW = DISABLE
    } else {
      pinMode(STOP_RIGHT_MOTOR, INPUT);     // FLOAT = ENABLE
    }
  }
  
  lastCommandTime = millis();
  
  DEBUG_PRINT("Motor DER: PWM=");
  DEBUG_PRINT(pwm);
  DEBUG_PRINT(" DIR=");
  DEBUG_PRINTLN(direction ? "FWD" : "BWD");
}

/**
 * CONFIGURAR AMBOS MOTORES SIMULTÁNEAMENTE
 */
void setBothMotors(int leftPwm, bool leftDir, int rightPwm, bool rightDir) {
  setLeftMotor(leftPwm, leftDir);
  setRightMotor(rightPwm, rightDir);
}

/**
 * DETENER AMBOS MOTORES
 */
void stopAllMotors() {
  setBothMotors(0, true, 0, true);
  DEBUG_PRINTLN("Todos los motores detenidos");
}

/**
 * HABILITAR/DESHABILITAR MOTOR IZQUIERDO
 */
void enableLeftMotor(bool enable) {
  leftMotor.enabled = enable;
  
  if (!enable) {
    analogWrite(PWM_LEFT_MOTOR, 0);
    pinMode(STOP_LEFT_MOTOR, OUTPUT);
    digitalWrite(STOP_LEFT_MOTOR, LOW); // LOW = DISABLE motor
  } else {
    pinMode(STOP_LEFT_MOTOR, INPUT);    // FLOAT = ENABLE motor
  }
  
  DEBUG_PRINT("Motor izquierdo ");
  DEBUG_PRINTLN(enable ? "habilitado" : "deshabilitado");
}

/**
 * HABILITAR/DESHABILITAR MOTOR DERECHO
 */
void enableRightMotor(bool enable) {
  rightMotor.enabled = enable;
  
  if (!enable) {
    analogWrite(PWM_RIGHT_MOTOR, 0);
    digitalWrite(STOP_RIGHT_MOTOR, LOW); // LOW = DISABLE motor
  }
  
  DEBUG_PRINT("Motor derecho ");
  DEBUG_PRINTLN(enable ? "habilitado" : "deshabilitado");
}

/**
 * HABILITAR/DESHABILITAR AMBOS MOTORES
 */
void enableAllMotors(bool enable) {
  enableLeftMotor(enable);
  enableRightMotor(enable);
}

/**
 * ACTIVAR/DESACTIVAR FRENO MOTOR IZQUIERDO
 */
void brakeLeftMotor(bool brake) {
  leftMotor.braked = brake;
  
  if (brake) {
    pinMode(BRAKE_LEFT_MOTOR, OUTPUT);
    digitalWrite(BRAKE_LEFT_MOTOR, HIGH);  // HIGH = BRAKE ACTIVO
    analogWrite(PWM_LEFT_MOTOR, 0);
  } else {
    pinMode(BRAKE_LEFT_MOTOR, INPUT);      // FLOAT = BRAKE DESACTIVADO
  }
  
  DEBUG_PRINT("Freno motor izquierdo ");
  DEBUG_PRINTLN(brake ? "activado" : "desactivado");
}

/**
 * ACTIVAR/DESACTIVAR FRENO MOTOR DERECHO
 */
void brakeRightMotor(bool brake) {
  rightMotor.braked = brake;
  
  if (brake) {
    pinMode(BRAKE_RIGHT_MOTOR, OUTPUT);
    digitalWrite(BRAKE_RIGHT_MOTOR, HIGH);  // HIGH = BRAKE ACTIVO
    analogWrite(PWM_RIGHT_MOTOR, 0);
  } else {
    pinMode(BRAKE_RIGHT_MOTOR, INPUT);      // FLOAT = BRAKE DESACTIVADO
  }
  
  DEBUG_PRINT("Freno motor derecho ");
  DEBUG_PRINTLN(brake ? "activado" : "desactivado");
}

/**
 * ACTIVAR/DESACTIVAR FRENOS DE AMBOS MOTORES
 */
void brakeAllMotors(bool brake) {
  brakeLeftMotor(brake);
  brakeRightMotor(brake);
}

//===========================================================================
//==================== FUNCIONES DE MOVIMIENTO BÁSICO ====================
//===========================================================================

/**
 * AVANZAR HACIA ADELANTE
 */
void moveForward(int speed) {
  speed = constrain(speed, 0, MAX_PWM_VALUE);
  setBothMotors(speed, dirForwardLeft, speed, dirForwardRight);
  DEBUG_PRINT("Avanzando a velocidad: ");
  DEBUG_PRINTLN(speed);
}

/**
 * RETROCEDER
 */
void moveBackward(int speed) {
  speed = constrain(speed, 0, MAX_PWM_VALUE);
  setBothMotors(speed, !dirForwardLeft, speed, !dirForwardRight);
  DEBUG_PRINT("Retrocediendo a velocidad: ");
  DEBUG_PRINTLN(speed);
}

/**
 * GIRAR A LA IZQUIERDA (DIFERENCIAL)
 */
void turnLeft(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, MAX_PWM_VALUE);
  rightSpeed = constrain(rightSpeed, 0, MAX_PWM_VALUE);
  
  setBothMotors(leftSpeed, true, rightSpeed, true);
  
  DEBUG_PRINT("Girando izquierda - IZQ:");
  DEBUG_PRINT(leftSpeed);
  DEBUG_PRINT(" DER:");
  DEBUG_PRINTLN(rightSpeed);
}

/**
 * GIRAR A LA DERECHA (DIFERENCIAL)
 */
void turnRight(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, MAX_PWM_VALUE);
  rightSpeed = constrain(rightSpeed, 0, MAX_PWM_VALUE);
  
  setBothMotors(leftSpeed, true, rightSpeed, true);
  
  DEBUG_PRINT("Girando derecha - IZQ:");
  DEBUG_PRINT(leftSpeed);
  DEBUG_PRINT(" DER:");
  DEBUG_PRINTLN(rightSpeed);
}

//===========================================================================
//==================== FUNCIONES DE SEGURIDAD ============================
//===========================================================================

/**
 * VERIFICACIÓN DE SEGURIDAD PERIÓDICA
 */
void motorSafetyCheck() {
  // Si el safety check está deshabilitado, salir
  if (!safetyCheckEnabled) return;
  
  unsigned long currentTime = millis();
  
  // Verificar timeout de comandos
  if (currentTime - lastCommandTime > EMERGENCY_STOP_TIME) {
    emergencyStop();
    DEBUG_PRINTLN("TIMEOUT - Parada de emergencia por falta de comandos");
  }
  
  // Otras verificaciones de seguridad pueden agregarse aquí
}

/**
 * OBTENER ESTADO DE LOS MOTORES
 */
void printMotorStatus() {
  DEBUG_PRINTLN("=== ESTADO DE MOTORES ===");
  
  DEBUG_PRINT("Motor Izquierdo - PWM:");
  DEBUG_PRINT(leftMotor.pwm);
  DEBUG_PRINT(" DIR:");
  DEBUG_PRINT(leftMotor.direction ? "FWD" : "BWD");
  DEBUG_PRINT(" STOP:");
  DEBUG_PRINT(leftMotor.stopped ? "SI" : "NO");
  DEBUG_PRINT(" BRAKE:");
  DEBUG_PRINT(leftMotor.braked ? "SI" : "NO");
  DEBUG_PRINT(" ENABLED:");
  DEBUG_PRINTLN(leftMotor.enabled ? "SI" : "NO");
  
  DEBUG_PRINT("Motor Derecho - PWM:");
  DEBUG_PRINT(rightMotor.pwm);
  DEBUG_PRINT(" DIR:");
  DEBUG_PRINT(rightMotor.direction ? "FWD" : "BWD");
  DEBUG_PRINT(" STOP:");
  DEBUG_PRINT(rightMotor.stopped ? "SI" : "NO");
  DEBUG_PRINT(" BRAKE:");
  DEBUG_PRINT(rightMotor.braked ? "SI" : "NO");
  DEBUG_PRINT(" ENABLED:");
  DEBUG_PRINTLN(rightMotor.enabled ? "SI" : "NO");
  
  DEBUG_PRINT("Último comando hace: ");
  DEBUG_PRINT((millis() - lastCommandTime) / 1000);
  DEBUG_PRINTLN(" segundos");
}

#endif // MOTOR_CONTROL_H