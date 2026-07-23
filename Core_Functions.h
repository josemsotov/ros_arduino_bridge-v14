/**
 * MOTOR-INTERFACE-V-13 CORE FUNCTIONS
 * Smart Golf Trolley - Funciones básicas del sistema
 */

#ifndef CORE_FUNCTIONS_H
#define CORE_FUNCTIONS_H

//===========================================================================
//======================= VARIABLES GLOBALES CORE =========================
//===========================================================================

/**
 * VARIABLES DE ESTADO DEL SISTEMA
 */
enum SystemMode { 
  MODE_PRUEBAS = 0, 
  MODE_LAZO_ABIERTO = 1, 
  MODE_LAZO_CERRADO = 2 
};

SystemMode currentMode = MODE_PRUEBAS;
bool systemInitialized = false;
unsigned long systemStartTime = 0;

/**
 * CONTADORES DE PULSOS GLOBALES
 */
#ifdef ENABLE_HALL_SENSORS
  volatile uint16_t leftHallCount = 0;
  volatile uint16_t rightHallCount = 0;
#endif

#ifdef ENABLE_OPTO_ENCODERS
  volatile uint16_t leftOptoCount = 0;
  volatile uint16_t rightOptoCount = 0;
#endif

/**
 * BUFFER DE COMUNICACIÓN SERIAL
 */
#ifdef ENABLE_SERIAL_COMMANDS
  String serialBuffer = "";
  bool commandReady = false;
#endif

//===========================================================================
//==================== DECLARACIONES FORWARD ==============================
//===========================================================================

// Declaraciones de funciones de otros módulos
void initializeMotorPins();  // Motor_Control.h
void forceStopPinsLowInit(); // Motor_Control.h - Forzar STOP pins a LOW en inicialización
void initializeHallSensors();  // Hall_Sensors.h
void initializeOptoEncoders();  // Motor_Control.h (función dummy si no hay opto)

//===========================================================================
//==================== FUNCIONES BÁSICAS DEL SISTEMA =====================
//===========================================================================

/**
 * INICIALIZACIÓN DEL SISTEMA
 */
void initializeSystem() {
  systemStartTime = millis();
  
  DEBUG_PRINTLN("MOTOR-INTERFACE-V-13 INICIANDO...");
  DEBUG_PRINTLN("Configuracion cargada desde Configuration.h");
  
  // Inicializar pines de motores
  initializeMotorPins();
  
  // *** FORZAR ESTADO STOP PINS A LOW (CRÍTICO) ***
  forceStopPinsLowInit();
  
  // *** ESTABLECER ESTADO INICIAL DEL ROBOT ***
  setStateInhabilitado();  // Robot comienza INHABILITADO por seguridad
  
  // Inicializar sensores según configuración
  #ifdef ENABLE_HALL_SENSORS
    initializeHallSensors();
  #endif
  
  #ifdef ENABLE_OPTO_ENCODERS
    initializeOptoEncoders();
  #endif
  
  // Inicializar MPU9250/6500 si está habilitado
  #ifdef ENABLE_MPU9250
    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("Inicializando I2C para MPU9250/6500...");
    Wire.begin();
    Wire.setClock(400000);  // 400kHz para máxima velocidad
    DEBUG_PRINTLN("I2C iniciado a 400kHz");
    
    mpu_initialize();
    mpu_configure();
    
    #ifdef ENABLE_MPU_AUTO_CALIBRATION
      mpu_calibrateOffsets();
    #endif
    
    DEBUG_PRINTLN("MPU9250/6500 listo!");
  #endif
  
  // Inicializar Joystick si está habilitado
  #ifdef ENABLE_JOYSTICK
    DEBUG_PRINTLN("");
    joy_initialize();
  #endif
  
  // Inicializar PID si está habilitado
  #ifdef ENABLE_PID_CONTROL
    // initializePID(); // Implementación básica - no requiere inicialización especial
  #endif
  
  systemInitialized = true;
  DEBUG_PRINTLN("Sistema inicializado correctamente");
  DEBUG_PRINTLN("Robot en estado: INHABILITADO - Use HABILITAR para activar");
}

// Función initializeMotorPins() eliminada - está definida en Motor_Control.h

/**
 * PARADA DE EMERGENCIA
 */
void emergencyStop() {
  // Detener PWM
  motor_pwm_write(PWM_LEFT_MOTOR, 0);
  motor_pwm_write(PWM_RIGHT_MOTOR, 0);
  
  // ACTIVAR frenos: BRAKE pins en HIGH
  pinMode(BRAKE_LEFT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_LEFT_MOTOR, HIGH);   // HIGH = BRAKE ACTIVO
  pinMode(BRAKE_RIGHT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_RIGHT_MOTOR, HIGH);  // HIGH = BRAKE ACTIVO
  
  // DESACTIVAR motores: STOP pins en LOW
  pinMode(STOP_LEFT_MOTOR, OUTPUT);
  digitalWrite(STOP_LEFT_MOTOR, LOW);     // LOW = DISABLE
  pinMode(STOP_RIGHT_MOTOR, OUTPUT);
  digitalWrite(STOP_RIGHT_MOTOR, LOW);    // LOW = DISABLE
  
  DEBUG_PRINTLN("PARADA DE EMERGENCIA ACTIVADA");
}

/**
 * LIBERACIÓN DE MOTORES
 */
void releaseMotors() {
  // LIBERAR frenos: BRAKE pins en FLOAT (HIGH IMPEDANCE)
  pinMode(BRAKE_LEFT_MOTOR, INPUT);   // FLOAT = BRAKE DESACTIVADO
  pinMode(BRAKE_RIGHT_MOTOR, INPUT);  // FLOAT = BRAKE DESACTIVADO
  
  // HABILITAR motores: STOP pins en FLOAT (HIGH IMPEDANCE)
  pinMode(STOP_LEFT_MOTOR, INPUT);    // FLOAT = ENABLE
  pinMode(STOP_RIGHT_MOTOR, INPUT);   // FLOAT = ENABLE
  
  DEBUG_PRINTLN("Motores liberados y habilitados (STOP y BRAKE pins en FLOAT)");
}

/**
 * FUNCIÓN DE RESET DEL SISTEMA
 */
void resetSystem() {
  DEBUG_PRINTLN("Reiniciando sistema...");
  
  // Parada de emergencia
  emergencyStop();
  
  // Reset contadores
  #ifdef ENABLE_HALL_SENSORS
    leftHallCount = 0;
    rightHallCount = 0;
  #endif
  
  #ifdef ENABLE_OPTO_ENCODERS
    leftOptoCount = 0;
    rightOptoCount = 0;
  #endif
  
  // Reset PID si está habilitado
  #ifdef ENABLE_PID_CONTROL
    // resetPID(); // Implementación básica - no requiere reset especial
  #endif
  
  // Reset controles
  

  
  // Volver a modo pruebas
  currentMode = MODE_PRUEBAS;
  
  DEBUG_PRINTLN("Sistema reiniciado");
}

/**
 * MONITOREO DEL SISTEMA
 */
void systemMonitoring() {
  static unsigned long lastMonitorTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastMonitorTime >= 1000) {  // Cada segundo
    lastMonitorTime = currentTime;
    
    #ifdef ENABLE_VERBOSE_OUTPUT
      DEBUG_PRINT("Uptime: ");
      DEBUG_PRINT((currentTime - systemStartTime) / 1000);
      DEBUG_PRINTLN(" s");
    #endif
    
    // Verificaciones de seguridad aquí si es necesario
  }
}

/**
 * OBTENER ESTADO DEL SISTEMA
 */
void printSystemInfo() {
  DEBUG_PRINTLN("=== INFORMACIÓN DEL SISTEMA ===");
  
  DEBUG_PRINT("Versión: MOTOR-INTERFACE-V-13");
  DEBUG_PRINTLN("");
  
  DEBUG_PRINT("Modo actual: ");
  switch(currentMode) {
    case MODE_PRUEBAS: DEBUG_PRINTLN("PRUEBAS"); break;
    case MODE_LAZO_ABIERTO: DEBUG_PRINTLN("LAZO ABIERTO"); break;
    case MODE_LAZO_CERRADO: DEBUG_PRINTLN("LAZO CERRADO"); break;
  }
  
  DEBUG_PRINT("Tiempo funcionamiento: ");
  DEBUG_PRINT((millis() - systemStartTime) / 1000);
  DEBUG_PRINTLN(" segundos");
  
  DEBUG_PRINTLN("Módulos activos:");
  #ifdef ENABLE_DUAL_CROSSING_CONTROL
    DEBUG_PRINTLN("  ✓ Control Dual de Cruce");
  #endif
  #ifdef ENABLE_PID_CONTROL
    DEBUG_PRINTLN("  ✓ Control PID");
  #endif


  #ifdef ENABLE_HALL_SENSORS
    DEBUG_PRINTLN("  ✓ Sensores Hall");
  #endif
  #ifdef ENABLE_OPTO_ENCODERS
    DEBUG_PRINTLN("  ✓ OptoEncoders");
  #endif
}

//===========================================================================
//==================== FUNCIONES BÁSICAS ADICIONALES =====================
//===========================================================================

/**
 * FUNCIONES BÁSICAS PARA MÓDULOS NO IMPLEMENTADOS COMPLETAMENTE
 * Estas funciones proporcionan funcionalidad básica cuando los módulos
 * específicos no están totalmente implementados
 */



#ifdef ENABLE_OPTO_ENCODERS
void updateOptoSpeeds() {
  // Implementación básica para OptoEncoders
  // TODO: Implementar lectura completa de OptoEncoders
  static unsigned long lastOptoUpdate = 0;
  if (millis() - lastOptoUpdate > 50) {  // Actualizar cada 50ms
    // Aquí iría la lógica básica de los OptoEncoders
    lastOptoUpdate = millis();
  }
}
#endif

#ifdef ENABLE_PID_CONTROL
void updatePIDControl() {
  // Implementación básica del control PID
  // TODO: Implementar control PID completo
  static unsigned long lastPIDUpdate = 0;
  if (millis() - lastPIDUpdate > PID_UPDATE_RATE) {
    // Aquí iría la lógica básica del PID
    lastPIDUpdate = millis();
  }
}
#endif

#endif // CORE_FUNCTIONS_H
