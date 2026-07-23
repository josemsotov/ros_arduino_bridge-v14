/**
 * MOTOR-INTERFACE-V-13 SERIAL COMMAND PROCESSOR
 * Smart Golf Trolley - Procesador de comandos serie estilo Marlin
 */

#ifndef SERIAL_COMMAND_PROCESSOR_H
#define SERIAL_COMMAND_PROCESSOR_H

#ifdef ENABLE_SERIAL_COMMANDS

//===========================================================================
//==================== DECLARACIONES DE FUNCIONES ========================
//===========================================================================

// Declaraciones forward para evitar errores de orden
void printCommandHelp();
void printUnknownCommand();
int extractNumber(String command, int defaultValue);
void extractTwoNumbers(String command, int &num1, int defaultNum1, int &num2, int defaultNum2);
// Funciones de sistemas avanzados eliminadas
void forceStopPinsHighImpedance();
void forceStopPinsLow();
void forceStopPinsFloat();
void showPinStatus();
void showCompleteSystemPinStatus();  // Nueva función expandible
void showStopPinStatus();
void showBrakePinStatus();
void forceAllPinsLow();
void forceAllPinsHigh();
void diagnosticPinSequence();

//===========================================================================
//==================== VARIABLES COMANDOS SERIE ==========================
//===========================================================================

#define MAX_COMMAND_LENGTH  50

// Variables del sistema de comandos serie (serialBuffer esta en Core_Functions.h)
extern String serialBuffer;  // Declaracion externa
extern bool commandReady;    // Declaracion externa

// Variables de debug y monitoreo
bool continuousDebugActive = false;
// Variables globales del procesador de comandos
unsigned long lastDebugOutput = 0;
unsigned long lastPotMonitorOutput = 0;

//===========================================================================
//==================== PROCESAMIENTO DE COMANDOS =========================
//===========================================================================

/**
 * LEER COMANDOS DEL PUERTO SERIE
 */
void readSerialCommands() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      if (serialBuffer.length() > 0) {
        commandReady = true;
      }
    } else if (inChar >= 32 && inChar <= 126) {  // Caracteres imprimibles
      if (serialBuffer.length() < MAX_COMMAND_LENGTH) {
        serialBuffer += inChar;
      }
    }
  }
}

/**
 * PROCESAR COMANDO RECIBIDO
 */
void processSerialCommand() {
  if (!commandReady) return;
  
  // Resetear timer de seguridad al recibir cualquier comando
  extern unsigned long lastCommandTime;
  lastCommandTime = millis();
  
  String cmd = serialBuffer;
  cmd.trim();
  
  // ===== VERIFICAR SI ES COMANDO ROS2 (ANTES DE UPPERCASE) =====
  #ifdef ENABLE_ROS2_BRIDGE
    if (ros2_tryProcessCommand(cmd)) {
      // Comando procesado por ROS2, no continuar
      serialBuffer = "";
      commandReady = false;
      return;
    }
  #endif
  
  // ===== CONVERTIR A MAYÚSCULAS PARA COMANDOS LOCALES =====
  cmd.toUpperCase();
  
  // ===== COMANDO DE PRUEBA =====
  if (cmd == "TEST" || cmd == "PRUEBA") {
    Serial.println("SISTEMA DE COMANDOS FUNCIONANDO CORRECTAMENTE");
    Serial.println("Comando TEST recibido y procesado OK");
  }
  // ===== COMANDOS BÁSICOS DEL SISTEMA =====
  else if (cmd == "INFO" || cmd == "STATUS") {
    printSystemInfo();
  }
  else if (cmd == "RESET") {
    resetSystem();
  }
  else if (cmd == "HELP") {
    printCommandHelp();
  }
  
  // ===== COMANDO DE CALIBRACIÓN PID GENERAL =====
  else if (cmd == "CALIBRATE" || cmd == "CALIBRACION" || cmd == "PID_CALIBRATE") {
    #ifdef PID_CONTROL_H
    pid_calibrate_general();
    #else
    Serial.println("PID_Control.h no incluido. No disponible.");
    #endif
  }
  // ===== COMANDO DE PRUEBA DIRECTA DE POSICIÓN (BYPASS PID) =====
  else if (cmd.startsWith("PTEST")) {
    Serial.println("🧪 PRUEBA DIRECTA DE POSICIÓN (SIN PID)");
    Serial.println("Habilitando robot...");
    setStateHabilitado();
    delay(100);
    Serial.println("Aplicando PWM=50 a ambos motores, dirección ADELANTE");
    // Usar lógica correcta de dirección (motor derecho invertido)
    setLeftMotor(50, true);   // true = HIGH = adelante para motor izquierdo
    setRightMotor(50, false); // false = LOW = adelante para motor derecho (invertido)
    Serial.println("✅ Motores activados con PWM=50");
    Serial.println("⏱️ Movimiento por 2 segundos...");
    delay(2000);
    Serial.println("🛑 Deteniendo...");
    setStateInhabilitado();
    Serial.println("✓ Prueba completada");
  }
  // ===== COMANDO DE PRUEBA VELOCIDAD CONTINUA (BYPASS PID) =====
  else if (cmd.startsWith("VTEST")) {
    Serial.println("🧪 PRUEBA VELOCIDAD CONTINUA (SIN PID)");
    Serial.println("Habilitando robot...");
    setStateHabilitado();
    delay(100);
    Serial.println("Aplicando PWM=30 a ambos motores continuamente");
    Serial.println("⚠️ El robot se moverá continuamente hasta que envíes STOP");
    // Usar lógica correcta de dirección (motor derecho invertido)
    setLeftMotor(30, true);   // true = HIGH = adelante para motor izquierdo
    setRightMotor(30, false); // false = LOW = adelante para motor derecho (invertido)
    Serial.println("✅ Motores activados con PWM=30");
    Serial.println("📢 Envía STOP para detener");
  }
  // ===== COMANDOS DE MOTORES BÁSICOS =====
  else if (cmd == "STOP" || cmd == "PARAR") {
    // Detener movimiento poniendo STOP pins en LOW
    setStateInhabilitado();
    DEBUG_PRINTLN("✓ Movimiento detenido - Robot INHABILITADO");
  }
  else if (cmd == "EMERGENCY" || cmd == "EMERGENCIA") {
    emergencyStop();
  }
  else if (cmd == "RELEASE" || cmd == "LIBERAR") {
    releaseMotors();
  }
  else if (cmd == "MOTOR_STATUS") {
    printMotorStatus();
  }
  else if (cmd == "SAFETY_ON") {
    extern bool safetyCheckEnabled;
    safetyCheckEnabled = true;
    Serial.println("Sistema de seguridad ACTIVADO");
  }
  else if (cmd == "SAFETY_OFF") {
    extern bool safetyCheckEnabled;
    safetyCheckEnabled = false;
    Serial.println("Sistema de seguridad DESACTIVADO");
  }
  
  // ===== COMANDOS DE ESTADOS DEL ROBOT =====
  else if (cmd == "INHABILITAR") {
    setStateInhabilitado();
  }
  else if (cmd == "BLOQUEAR") {
    setStateBloqueado();
  }
  else if (cmd == "HABILITAR") {
    setStateHabilitado();
  }
  else if (cmd == "ESTADO" || cmd == "STATE") {
    printRobotState();
  }
  
  // ===== COMANDO DE PRUEBA TOTAL =====
  else if (cmd == "PRUEBA-TOTAL" || cmd == "TEST-TOTAL") {
    startPruebaTotal();
  }
  
  // ===== COMANDOS DE MOVIMIENTO CON CONTROL DE ESTADO =====
  else if (cmd == "ADELANTE") {
    moveForwardState(DEFAULT_PWM_FORWARD);
  }
  else if (cmd.startsWith("AD")) {
    int pwm = extractNumber(cmd, DEFAULT_PWM_FORWARD);
    moveForwardState(pwm);
  }
  else if (cmd == "ATRAS") {
    moveBackwardState(DEFAULT_PWM_BACKWARD);
  }
  else if (cmd.startsWith("AT")) {
    int pwm = extractNumber(cmd, DEFAULT_PWM_BACKWARD);
    moveBackwardState(pwm);
  }
  
  // ===== COMANDOS DE CRUCE POR ÁNGULO (NUEVO) =====
  // Formato: CRUCE [ángulo] [pwm]
  // El ángulo positivo gira a la DERECHA, negativo a la IZQUIERDA
  // Sistema calcula automáticamente los pulsos basándose en geometría del robot
  // 
  // Ejemplos: 
  //   CRUCE 45     → Gira 45° derecha, 20 PWM
  //   CRUCE 45 40  → Gira 45° derecha, 40 PWM
  //   CRUCE -45    → Gira 45° izquierda, 20 PWM
  //   CRUCE -90 60 → Gira 90° izquierda, 60 PWM
  else if (cmd.startsWith("CRUCE ") || cmd == "CRUCE") {
    float angle;
    int pwm;
    
    // Extraer ángulo y PWM
    int spaceIndex = cmd.indexOf(' ', 6);  // Buscar espacio después de "CRUCE "
    
    if (spaceIndex > 0) {
      // Hay parámetros
      String params = cmd.substring(6);  // Todo después de "CRUCE "
      params.trim();
      
      int secondSpace = params.indexOf(' ');
      if (secondSpace > 0) {
        // Hay dos parámetros: ángulo y PWM
        angle = params.substring(0, secondSpace).toFloat();
        pwm = params.substring(secondSpace + 1).toInt();
        if (pwm == 0) pwm = DEFAULT_PWM_TURN;  // Si no se especifica PWM
      } else {
        // Solo ángulo
        angle = params.toFloat();
        pwm = DEFAULT_PWM_TURN;
      }
    } else {
      // Sin parámetros - usar defaults
      angle = DEFAULT_TURN_ANGLE;
      pwm = DEFAULT_PWM_TURN;
    }
    
    // Determinar dirección basándose en signo del ángulo
    if (angle >= 0) {
      DEBUG_PRINT("🔄 Ejecutando CRUCE DERECHA: ");
      DEBUG_PRINT(angle);
      DEBUG_PRINT("° a ");
      DEBUG_PRINT(pwm);
      DEBUG_PRINTLN(" PWM");
      executeCruceAngleDerecha(angle, pwm);
    } else {
      DEBUG_PRINT("🔄 Ejecutando CRUCE IZQUIERDA: ");
      DEBUG_PRINT(abs(angle));
      DEBUG_PRINT("° a ");
      DEBUG_PRINT(pwm);
      DEBUG_PRINTLN(" PWM");
      executeCruceAngleIzquierda(abs(angle), pwm);
    }
  }
  
  // ===== COMANDOS DE CRUCE (GIROS PUNTUALES) - MANTENER COMPATIBILIDAD =====
  // Formato: CRUCE-DER [pulsos] [pwm]
  // Ejemplos: CRUCE-DER → usa defaults (54 pulsos, 20 PWM)
  //           CRUCE-DER 45 → 45 pulsos, 20 PWM
  //           CRUCE-DER 45 60 → 45 pulsos, 60 PWM
  else if (cmd.startsWith("CRUCE-DER")) {
    int pulsos, pwm;
    extractTwoNumbers(cmd, pulsos, DEFAULT_HALL_PULSES_TURN, pwm, DEFAULT_PWM_TURN);
    executeCruceDerecha(pwm, pulsos);
  }
  else if (cmd.startsWith("CRUCE-IZQ")) {
    int pulsos, pwm;
    extractTwoNumbers(cmd, pulsos, DEFAULT_HALL_PULSES_TURN, pwm, DEFAULT_PWM_TURN);
    executeCruceIzquierda(pwm, pulsos);
  }
  else if (cmd.startsWith("L")) {
    int pwm = extractNumber(cmd, 0);
    setLeftMotor(pwm, true);    // true = HIGH = adelante motor izquierdo
  }
  else if (cmd.startsWith("R")) {
    int pwm = extractNumber(cmd, 0);
    setRightMotor(pwm, false);  // false = LOW = adelante motor derecho (DIR eléctrico invertido)
  }
  
  // ===== COMANDOS DE CONTROL AVANZADO (NO DISPONIBLES) =====
  
  // ===== COMANDOS DE CONTROL AVANZADO ELIMINADOS =====
  
  // ===== COMANDOS SENSORES HALL =====
  #ifdef ENABLE_HALL_SENSORS
  else if (cmd == "HALL_INFO") {
    printHallInfo();
  }
  else if (cmd == "HALL_DEBUG") {
    continuousDebugActive = true;
    DEBUG_PRINTLN("Debug continuo Hall activado");
  }
  else if (cmd == "HALL_TEST") {
    testHallSensors();
  }
  else if (cmd == "HALL_RESET") {
    resetHallCounters();
  }
  #endif
  
  #ifdef ENABLE_JOYSTICK
  else if (cmd == "JOY_STATUS" || cmd == "JOY_INFO") {
    joy_printStatus();
  }
  else if (cmd == "JOY_DATA" || cmd == "JOY_RAW") {
    joy_printRawData();
  }
  else if (cmd == "JOY_TEST") {
    joy_runHardwareTest();
  }
  else if (cmd == "JOY_MONITOR_START" || cmd == "JOY_START") {
    joy_startContinuousMonitor();
  }
  else if (cmd == "JOY_MONITOR_STOP" || cmd == "JOY_STOP") {
    joy_stopContinuousMonitor();
  }
  else if (cmd == "JOY_TANK_ON") {
    setJoystickTankMode(true);
  }
  else if (cmd == "JOY_TANK_OFF") {
    setJoystickTankMode(false);
  }
  else if (cmd == "JOY_RESET") {
    joy_resetTankMode();
  }
  #endif
  
  // ===== COMANDOS DE DEBUG GENERAL =====
  else if (cmd == "DEBUG_ON") {
    continuousDebugActive = true;
    DEBUG_PRINTLN("Debug continuo activado");
  }
  else if (cmd == "DEBUG_OFF" || cmd == "STOP_DEBUG") {
    continuousDebugActive = false;
    DEBUG_PRINTLN("Debug continuo desactivado");
  }
  
  // ===== COMANDOS DE ESTADO DE PINES =====
  else if (cmd == "SYSTEM_PINS" || cmd == "ALL_PINS" || cmd == "PINS_COMPLETE") {
    showCompleteSystemPinStatus();
  }
  
  // ===== COMANDO NO RECONOCIDO =====
  else {
    printUnknownCommand();
  }
  
  // Limpiar buffer
  serialBuffer = "";
  commandReady = false;
}

/**
 * EXTRAER NUMERO DE UN COMANDO
 */
int extractNumber(String command, int defaultValue) {
  // Buscar el primer digito
  int startIndex = -1;
  for (int i = 0; i < command.length(); i++) {
    if (isDigit(command.charAt(i))) {
      startIndex = i;
      break;
    }
  }
  
  if (startIndex == -1) {
    return defaultValue;
  }
  
  // Extraer el numero
  String numberStr = "";
  for (int i = startIndex; i < command.length(); i++) {
    char c = command.charAt(i);
    if (isDigit(c)) {
      numberStr += c;
    } else {
      break;
    }
  }
  
  return numberStr.toInt();
}

/**
 * EXTRAER DOS NÚMEROS DE UN COMANDO
 * Formato: COMANDO NUM1 NUM2
 * Ejemplo: "CRUCE-DER 45 60" → num1=45, num2=60
 */
void extractTwoNumbers(String command, int &num1, int defaultNum1, int &num2, int defaultNum2) {
  num1 = defaultNum1;
  num2 = defaultNum2;
  
  // Buscar el primer número
  int startIndex1 = -1;
  for (int i = 0; i < command.length(); i++) {
    if (isDigit(command.charAt(i))) {
      startIndex1 = i;
      break;
    }
  }
  
  if (startIndex1 == -1) {
    return;  // No hay números
  }
  
  // Extraer primer número
  String numberStr1 = "";
  int i = startIndex1;
  while (i < command.length() && isDigit(command.charAt(i))) {
    numberStr1 += command.charAt(i);
    i++;
  }
  num1 = numberStr1.toInt();
  
  // Buscar el segundo número (después de un espacio o no-dígito)
  int startIndex2 = -1;
  for (int j = i; j < command.length(); j++) {
    if (isDigit(command.charAt(j))) {
      startIndex2 = j;
      break;
    }
  }
  
  if (startIndex2 == -1) {
    return;  // Solo hay un número
  }
  
  // Extraer segundo número
  String numberStr2 = "";
  for (int j = startIndex2; j < command.length(); j++) {
    char c = command.charAt(j);
    if (isDigit(c)) {
      numberStr2 += c;
    } else {
      break;
    }
  }
  num2 = numberStr2.toInt();
}

/**
 * MOSTRAR AYUDA DE COMANDOS
 */
void printCommandHelp() {
  DEBUG_PRINTLN("=== COMANDOS MOTOR-INTERFACE-V-13 CON ESTADOS ===");
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("⚠️ ESTADO INICIAL: MOTORES INHABILITADOS");
  DEBUG_PRINTLN("   Los motores inician DESHABILITADOS por seguridad");
  DEBUG_PRINTLN("   Usa HABILITAR para activar el sistema");
  DEBUG_PRINTLN("");
  
  DEBUG_PRINTLN("🔄 ESTADOS DEL ROBOT:");
  DEBUG_PRINTLN("  INHABILITAR - STOP=LOW, BRAKE=FLOAT, PWM=0");
  DEBUG_PRINTLN("  BLOQUEAR - STOP=LOW, BRAKE=HIGH, PWM=0");
  DEBUG_PRINTLN("  HABILITAR - Activar sistema para comandos (LAZO ABIERTO)");
  DEBUG_PRINTLN("  ESTADO - Ver estado actual del robot");
  DEBUG_PRINTLN("");
  
  DEBUG_PRINTLN("🏠 SISTEMA:");
  DEBUG_PRINTLN("  INFO/STATUS - Estado del sistema");
  DEBUG_PRINTLN("  RESET/RESTART - Reiniciar sistema");
  DEBUG_PRINTLN("  HELP/? - Esta ayuda");
  DEBUG_PRINTLN("  STOP - Detener movimiento (STOP=LOW)");
  DEBUG_PRINTLN("");
  
  DEBUG_PRINTLN("� MOVIMIENTO (Solo en estado HABILITADO):");
  DEBUG_PRINTLN("  ADELANTE - Avanzar a 30 PWM (genérico)");
  DEBUG_PRINTLN("  AD [pwm] - Avanzar a PWM específico (ej: AD 40)");
  DEBUG_PRINTLN("  ATRAS - Retroceder a 30 PWM (genérico)");
  DEBUG_PRINTLN("  AT [pwm] - Retroceder a PWM específico (ej: AT 40)");
  DEBUG_PRINTLN("");
  
  DEBUG_PRINTLN("🔄 CRUCES/GIROS POR ÁNGULO (NUEVO - Solo con sensores Hall):");
  DEBUG_PRINTLN("  🎯 Sistema basado en geometría del robot:");
  DEBUG_PRINTLN("     • Entre-ejes: 82 cm | Diámetro ruedas: 20 cm");
  DEBUG_PRINTLN("     • Factor: 65.631 pulsos por grado de rotación");
  DEBUG_PRINTLN("  CRUCE [ángulo] - Girar ángulo especificado (+ derecha, - izquierda)");
  DEBUG_PRINTLN("  CRUCE [ángulo] [pwm] - Con PWM específico");
  DEBUG_PRINTLN("  Ejemplos:");
  DEBUG_PRINTLN("    CRUCE 45      → Gira 45° derecha (2954 pulsos), 20 PWM");
  DEBUG_PRINTLN("    CRUCE 45 40   → Gira 45° derecha a 40 PWM");
  DEBUG_PRINTLN("    CRUCE -45     → Gira 45° izquierda, 20 PWM");
  DEBUG_PRINTLN("    CRUCE -90 60  → Gira 90° izquierda a 60 PWM");
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("🔄 CRUCES/GIROS POR PULSOS (Compatibilidad):");
  DEBUG_PRINTLN("  CRUCE-DER - Girar derecha (54 pulsos, 20 PWM)");
  DEBUG_PRINTLN("  CRUCE-DER [pulsos] - Ejemplo: CRUCE-DER 45");
  DEBUG_PRINTLN("  CRUCE-DER [pulsos] [pwm] - Ejemplo: CRUCE-DER 45 60");
  DEBUG_PRINTLN("  CRUCE-IZQ - Girar izquierda (54 pulsos, 20 PWM)");
  DEBUG_PRINTLN("  CRUCE-IZQ [pulsos] - Ejemplo: CRUCE-IZQ 45");
  DEBUG_PRINTLN("  CRUCE-IZQ [pulsos] [pwm] - Ejemplo: CRUCE-IZQ 45 60");
  DEBUG_PRINTLN("");
  
  #ifdef ENABLE_HALL_SENSORS
  DEBUG_PRINTLN("📊 SENSORES HALL:");
  DEBUG_PRINTLN("  HALL - Lecturas actuales");
  DEBUG_PRINTLN("  HALL_TEST - Prueba de sensores");
  DEBUG_PRINTLN("  HALL_RESET - Resetear contadores");
  DEBUG_PRINTLN("  HALL_PPR - Mostrar configuración PPR");
  DEBUG_PRINTLN("");
  #endif
  
  #ifdef ENABLE_JOYSTICK
  DEBUG_PRINTLN("🎮 JOYSTICK DE CONTROL:");
  DEBUG_PRINTLN("  JOY_STATUS/JOY_INFO - Estado completo del joystick");
  DEBUG_PRINTLN("  JOY_DATA/JOY_RAW - Datos RAW actuales");
  DEBUG_PRINTLN("  JOY_TEST - Test de hardware (10 lecturas)");
  DEBUG_PRINTLN("  JOY_MONITOR_START/JOY_START - Monitoreo continuo");
  DEBUG_PRINTLN("  JOY_MONITOR_STOP/JOY_STOP - Detener monitoreo");
  DEBUG_PRINTLN("  JOY_TANK_ON - Activar modo tanque");
  DEBUG_PRINTLN("  JOY_TANK_OFF - Desactivar modo tanque");
  DEBUG_PRINTLN("  JOY_RESET - Resetear modo tanque");
  DEBUG_PRINTLN("");
  #endif
  
  DEBUG_PRINTLN("🔍 DEBUG Y DIAGNÓSTICO:");
  DEBUG_PRINTLN("  DEBUG_ON - Activar debug continuo");
  DEBUG_PRINTLN("  DEBUG_OFF/STOP_DEBUG - Desactivar debug");
  DEBUG_PRINTLN("  PIN_STATUS - Estado de pines de motores");
  DEBUG_PRINTLN("  SYSTEM_PINS/ALL_PINS - Estado COMPLETO de TODOS los pines");
  DEBUG_PRINTLN("  DIAGNOSTIC_PINS - Secuencia diagnóstico completo");
  DEBUG_PRINTLN("");
  
  DEBUG_PRINTLN("⚡ CONTROL DE PINES:");
  DEBUG_PRINTLN("  FORCE_STOP_LOW - STOPs a LOW (DISABLE motores)");
  DEBUG_PRINTLN("  FORCE_STOP_FLOAT - STOPs a FLOAT (ENABLE motores)");
  DEBUG_PRINTLN("  FORCE_STOP_ENABLE - Habilitar motores");
  DEBUG_PRINTLN("  FORCE_ALL_LOW - Todos los pines a LOW");
  DEBUG_PRINTLN("  FORCE_ALL_HIGH - Todos los pines a HIGH");
  DEBUG_PRINTLN("");
  
  DEBUG_PRINTLN("🧪 PRUEBAS:");
  DEBUG_PRINTLN("  VOLTAGE - Voltaje del sistema");
  DEBUG_PRINTLN("  CURRENT - Consumo de corriente");
  DEBUG_PRINTLN("");
  
  DEBUG_PRINTLN("⚠️ ELIMINADOS: Tank Control, Control Diferencial, Potenciómetros");
  DEBUG_PRINTLN("Solo funciones básicas de motor y sensores disponibles.");
}

/**
 * COMANDO NO RECONOCIDO
 */
void printUnknownCommand() {
  DEBUG_PRINTLN("ERROR: COMANDO NO RECONOCIDO");
  DEBUG_PRINTLN("Usa 'HELP' para ver comandos disponibles");
}

/**
 * PROCESAR DEBUG CONTINUO
 */
void processContinuousDebug() {
  if (!continuousDebugActive) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastDebugOutput >= 500) {  // Cada 500ms
    lastDebugOutput = currentTime;
    
    // Debug de sistemas avanzados eliminado
    
    #ifdef ENABLE_HALL_SENSORS
    printHallDebug();
    #endif
  }
}

//===========================================================================
//==================== FUNCIONES AUXILIARES FALTANTES ====================
// Duplicacion eliminada - extractNumber ya esta definido arriba

/**
 * FUNCIONES DE SISTEMAS AVANZADOS ELIMINADAS
 * Solo funciones básicas de motor y sensores disponibles
 */

/**
 * FORZAR PINES STOP A HIGH IMPEDANCE (ENABLE)
 */
void forceStopPinsHighImpedance() {
  Serial.println("=== FORZANDO PINES STOP A HIGH IMPEDANCE ===");
  Serial.println("Configurando pines STOP para habilitar motores...");
  Serial.println("");
  
  // Mostrar estado actual
  Serial.println("Estado ANTES:");
  Serial.print("STOP_LEFT_MOTOR (pin ");
  Serial.print(STOP_LEFT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(STOP_LEFT_MOTOR) ? "HIGH" : "LOW");
  
  Serial.print("STOP_RIGHT_MOTOR (pin ");
  Serial.print(STOP_RIGHT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(STOP_RIGHT_MOTOR) ? "HIGH" : "LOW");
  Serial.println("");
  
  // FORZAR a HIGH IMPEDANCE (INPUT)
  Serial.println("Configurando pines STOP como INPUT (HIGH IMPEDANCE)...");
  pinMode(STOP_LEFT_MOTOR, INPUT);
  pinMode(STOP_RIGHT_MOTOR, INPUT);
  
  // Pequeña pausa para estabilizar
  delay(100);
  
  // Verificar estado después
  Serial.println("Estado DESPUÉS:");
  Serial.print("STOP_LEFT_MOTOR (pin ");
  Serial.print(STOP_LEFT_MOTOR);
  Serial.print("): ");
  Serial.print(digitalRead(STOP_LEFT_MOTOR) ? "HIGH" : "LOW");
  Serial.println(" (INPUT/HIGH IMPEDANCE)");
  
  Serial.print("STOP_RIGHT_MOTOR (pin ");
  Serial.print(STOP_RIGHT_MOTOR);
  Serial.print("): ");
  Serial.print(digitalRead(STOP_RIGHT_MOTOR) ? "HIGH" : "LOW");
  Serial.println(" (INPUT/HIGH IMPEDANCE)");
  Serial.println("");
  
  // Configurar también otros pines para motores activos
  Serial.println("Configurando otros pines para motores ACTIVOS:");
  
  // Liberar frenos
  pinMode(BRAKE_LEFT_MOTOR, INPUT);   // FLOAT = BRAKE DESACTIVADO
  pinMode(BRAKE_RIGHT_MOTOR, INPUT);  // FLOAT = BRAKE DESACTIVADO
  Serial.println("✓ Frenos LIBERADOS (BRAKE pins = FLOAT)");
  
  // Configurar direcciones
  pinMode(DIR_LEFT_MOTOR, OUTPUT);
  pinMode(DIR_RIGHT_MOTOR, OUTPUT);
  digitalWrite(DIR_LEFT_MOTOR, DIR_FORWARD_LEFT);
  digitalWrite(DIR_RIGHT_MOTOR, DIR_FORWARD_RIGHT);
  Serial.println("✓ Direcciones configuradas para ADELANTE");
  
  // Configurar PWM
  pinMode(PWM_LEFT_MOTOR, OUTPUT);
  pinMode(PWM_RIGHT_MOTOR, OUTPUT);
  motor_pwm_write(PWM_LEFT_MOTOR, 0);
  motor_pwm_write(PWM_RIGHT_MOTOR, 0);
  Serial.println("✓ PWM configurado (PWM pins = 0 = PARADO)");
  
  Serial.println("");
  Serial.println("=== CONFIGURACIÓN COMPLETADA ===");
  Serial.println("Los motores ahora deberían estar HABILITADOS");
  Serial.println("- STOP pins: HIGH IMPEDANCE (ENABLE)");
  Serial.println("- BRAKE pins: HIGH IMPEDANCE (SIN FRENO)");
  Serial.println("- DIR pins: ADELANTE (izq=HIGH, der=LOW)");
  Serial.println("- PWM pins: 0 (PARADO)");
  Serial.println("");
  Serial.println("Prueba con comandos como:");
  Serial.println("- ADELANTE30");
  Serial.println("- L20 (motor izquierdo)");
  Serial.println("- R20 (motor derecho)");
  Serial.println("");
  
  // Actualizar estados internos de motores si existen
  extern MotorState leftMotor, rightMotor;
  leftMotor.enabled = true;
  leftMotor.braked = false;
  leftMotor.stopped = true;
  leftMotor.pwm = 0;
  leftMotor.direction = (DIR_FORWARD_LEFT == HIGH);
  
  rightMotor.enabled = true;
  rightMotor.braked = false;
  rightMotor.stopped = true;
  rightMotor.pwm = 0;
  rightMotor.direction = (DIR_FORWARD_RIGHT == HIGH);
  
  Serial.println("Estados internos de motores actualizados.");
}

/**
 * FORZAR PINES STOP A LOW (DISABLE)
 */
void forceStopPinsLow() {
  Serial.println("=== FORZANDO PINES STOP A LOW (DISABLE) ===");
  Serial.println("Configurando pines STOP para deshabilitar motores...");
  Serial.println("");
  
  // Mostrar estado ANTES
  Serial.println("Estado ANTES:");
  showStopPinStatus();
  
  // FORZAR a LOW (OUTPUT)
  Serial.println("Configurando pines STOP como OUTPUT LOW...");
  pinMode(STOP_LEFT_MOTOR, OUTPUT);
  digitalWrite(STOP_LEFT_MOTOR, LOW);
  pinMode(STOP_RIGHT_MOTOR, OUTPUT);
  digitalWrite(STOP_RIGHT_MOTOR, LOW);
  
  delay(100);
  
  // Mostrar estado DESPUÉS
  Serial.println("Estado DESPUÉS:");
  showStopPinStatus();
  
  Serial.println("");
  Serial.println("✓ STOP pins configurados en LOW (motores DESHABILITADOS)");
  Serial.println("Los motores NO deberían moverse aunque tengan PWM");
  Serial.println("");
}

/**
 * FORZAR PINES STOP A FLOAT (ENABLE)
 */
void forceStopPinsFloat() {
  Serial.println("=== FORZANDO PINES STOP A FLOAT (ENABLE) ===");
  Serial.println("Configurando pines STOP para habilitar motores...");
  Serial.println("");
  
  // Mostrar estado ANTES
  Serial.println("Estado ANTES:");
  showStopPinStatus();
  
  // FORZAR a FLOAT (INPUT)
  Serial.println("Configurando pines STOP como INPUT (FLOAT)...");
  pinMode(STOP_LEFT_MOTOR, INPUT);
  pinMode(STOP_RIGHT_MOTOR, INPUT);
  
  delay(100);
  
  // Mostrar estado DESPUÉS
  Serial.println("Estado DESPUÉS:");
  showStopPinStatus();
  
  Serial.println("");
  Serial.println("✓ STOP pins configurados en FLOAT (motores HABILITADOS)");
  Serial.println("Los motores deberían poder moverse con PWM");
  Serial.println("");
}

/**
 * MOSTRAR ESTADO ACTUAL DE TODOS LOS PINES
 */
void showPinStatus() {
  Serial.println("=== ESTADO ACTUAL DE PINES DE MOTORES ===");
  Serial.println("");
  
  Serial.println("PINES STOP (ENABLE/DISABLE):");
  showStopPinStatus();
  Serial.println("");
  
  Serial.println("PINES BRAKE (FRENO):");
  showBrakePinStatus();
  Serial.println("");
  
  Serial.println("PINES DIRECCION:");
  Serial.print("DIR_LEFT_MOTOR (pin ");
  Serial.print(DIR_LEFT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(DIR_LEFT_MOTOR) ? "HIGH (ADELANTE)" : "LOW (ATRAS)");
  
  Serial.print("DIR_RIGHT_MOTOR (pin ");
  Serial.print(DIR_RIGHT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(DIR_RIGHT_MOTOR) ? "HIGH (ADELANTE)" : "LOW (ATRAS)");
  Serial.println("");
  
  Serial.println("PINES PWM:");
  Serial.print("PWM_LEFT_MOTOR (pin ");
  Serial.print(PWM_LEFT_MOTOR);
  Serial.print("): ");
  Serial.println(analogRead(PWM_LEFT_MOTOR));
  
  Serial.print("PWM_RIGHT_MOTOR (pin ");
  Serial.print(PWM_RIGHT_MOTOR);
  Serial.print("): ");
  Serial.println(analogRead(PWM_RIGHT_MOTOR));
  Serial.println("");
  
  Serial.println("ESTADOS INTERNOS:");
  extern MotorState leftMotor, rightMotor;
  Serial.print("leftMotor - enabled:");
  Serial.print(leftMotor.enabled ? "SI" : "NO");
  Serial.print(" braked:");
  Serial.print(leftMotor.braked ? "SI" : "NO");
  Serial.print(" stopped:");
  Serial.print(leftMotor.stopped ? "SI" : "NO");
  Serial.print(" pwm:");
  Serial.println(leftMotor.pwm);
  
  Serial.print("rightMotor - enabled:");
  Serial.print(rightMotor.enabled ? "SI" : "NO");
  Serial.print(" braked:");
  Serial.print(rightMotor.braked ? "SI" : "NO");
  Serial.print(" stopped:");
  Serial.print(rightMotor.stopped ? "SI" : "NO");
  Serial.print(" pwm:");
  Serial.println(rightMotor.pwm);
  Serial.println("");
}

/**
 * MOSTRAR ESTADO COMPLETO DE TODOS LOS PINES DEL SISTEMA
 * FUNCIÓN EXPANDIBLE - Se actualiza automáticamente cuando se agregan nuevos pines
 */
void showCompleteSystemPinStatus() {
  Serial.println("===============================================");
  Serial.println("🔍 ESTADO COMPLETO DE PINES DEL SISTEMA");
  Serial.println("===============================================");
  Serial.println("");
  
  // 🔧 SECCIÓN: PINES DE MOTORES
  Serial.println("🔧 PINES DE CONTROL DE MOTORES:");
  Serial.println("───────────────────────────────────────────────");
  
  // Pines PWM
  Serial.println("📊 PWM (Velocidad):");
  Serial.print("  PWM_LEFT_MOTOR  (Pin ");
  Serial.print(PWM_LEFT_MOTOR);
  Serial.print("): ");
  Serial.print(analogRead(PWM_LEFT_MOTOR));
  Serial.println(" (0-1023)");
  
  Serial.print("  PWM_RIGHT_MOTOR (Pin ");
  Serial.print(PWM_RIGHT_MOTOR);
  Serial.print("): ");
  Serial.print(analogRead(PWM_RIGHT_MOTOR));
  Serial.println(" (0-1023)");
  Serial.println("");
  
  // Pines de dirección
  Serial.println("➡️ DIRECCIÓN:");
  Serial.print("  DIR_LEFT_MOTOR  (Pin ");
  Serial.print(DIR_LEFT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(DIR_LEFT_MOTOR) ? "HIGH (ADELANTE)" : "LOW (ATRÁS)");
  
  Serial.print("  DIR_RIGHT_MOTOR (Pin ");
  Serial.print(DIR_RIGHT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(DIR_RIGHT_MOTOR) ? "HIGH (ADELANTE)" : "LOW (ATRÁS)");
  Serial.println("");
  
  // Pines STOP (ENABLE/DISABLE)
  Serial.println("🛑 PINES STOP (ENABLE/DISABLE):");
  Serial.print("  STOP_LEFT_MOTOR  (Pin ");
  Serial.print(STOP_LEFT_MOTOR);
  Serial.print("): ");
  pinMode(STOP_LEFT_MOTOR, INPUT_PULLUP);
  int stopLeftRead = digitalRead(STOP_LEFT_MOTOR);
  Serial.print(stopLeftRead ? "HIGH" : "LOW");
  Serial.println(stopLeftRead ? " (FLOTANTE/ENABLE)" : " (DISABLE)");
  
  Serial.print("  STOP_RIGHT_MOTOR (Pin ");
  Serial.print(STOP_RIGHT_MOTOR);
  Serial.print("): ");
  pinMode(STOP_RIGHT_MOTOR, INPUT_PULLUP);
  int stopRightRead = digitalRead(STOP_RIGHT_MOTOR);
  Serial.print(stopRightRead ? "HIGH" : "LOW");
  Serial.println(stopRightRead ? " (FLOTANTE/ENABLE)" : " (DISABLE)");
  Serial.println("");
  
  // Pines BRAKE (FRENO)
  Serial.println("🔒 PINES BRAKE (FRENO):");
  Serial.print("  BRAKE_LEFT_MOTOR  (Pin ");
  Serial.print(BRAKE_LEFT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(BRAKE_LEFT_MOTOR) ? "HIGH (ACTIVO)" : "LOW (INACTIVO)");
  
  Serial.print("  BRAKE_RIGHT_MOTOR (Pin ");
  Serial.print(BRAKE_RIGHT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(BRAKE_RIGHT_MOTOR) ? "HIGH (ACTIVO)" : "LOW (INACTIVO)");
  Serial.println("");
  
  // 📊 SECCIÓN: SENSORES (EXPANDIBLE)
  Serial.println("📊 PINES DE SENSORES:");
  Serial.println("───────────────────────────────────────────────");
  
  #ifdef ENABLE_HALL_SENSORS
  Serial.println("🧲 SENSORES HALL (Velocidad):");
  Serial.print("  HALL_LEFT_MOTOR  (Pin ");
  Serial.print(HALL_LEFT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(HALL_LEFT_MOTOR) ? "HIGH" : "LOW");
  
  Serial.print("  HALL_RIGHT_MOTOR (Pin ");
  Serial.print(HALL_RIGHT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(HALL_RIGHT_MOTOR) ? "HIGH" : "LOW");
  
  // Mostrar contadores de Hall si están disponibles
  extern volatile unsigned long leftHallCount, rightHallCount;
  Serial.print("  Contadores Hall: Izq=");
  Serial.print(leftHallCount);
  Serial.print(", Der=");
  Serial.println(rightHallCount);
  Serial.println("");
  #endif
  
  #ifdef ENABLE_OPTO_ENCODERS
  Serial.println("👁️ OPTOENCODERS (Posición):");
  Serial.print("  OPTO_LEFT_MOTOR  (Pin ");
  Serial.print(OPTO_LEFT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(OPTO_LEFT_MOTOR) ? "HIGH" : "LOW");
  
  Serial.print("  OPTO_RIGHT_MOTOR (Pin ");
  Serial.print(OPTO_RIGHT_MOTOR);
  Serial.print("): ");
  Serial.println(digitalRead(OPTO_RIGHT_MOTOR) ? "HIGH" : "LOW");
  Serial.println("");
  #endif
  
  // 🎛️ SECCIÓN: PINES DE CONTROL ADICIONALES (EXPANDIBLE PARA NUEVAS FUNCIONES)
  Serial.println("🎛️ PINES DE CONTROL ADICIONALES:");
  Serial.println("───────────────────────────────────────────────");
  
  // Esta sección se expandirá automáticamente cuando agregues nuevas funciones
  // TODO: Aquí se agregarán automáticamente nuevos pines cuando implementes más funciones
  Serial.println("  (Ningún pin adicional configurado actualmente)");
  Serial.println("");
  
  // 💾 SECCIÓN: ESTADOS INTERNOS DEL SISTEMA
  Serial.println("💾 ESTADOS INTERNOS DEL SISTEMA:");
  Serial.println("───────────────────────────────────────────────");
  
  extern MotorState leftMotor, rightMotor;
  Serial.println("🔧 Estado Motor Izquierdo:");
  Serial.print("  Habilitado: ");
  Serial.println(leftMotor.enabled ? "✅ SÍ" : "❌ NO");
  Serial.print("  Frenado: ");
  Serial.println(leftMotor.braked ? "🔒 SÍ" : "🔓 NO");
  Serial.print("  Detenido: ");
  Serial.println(leftMotor.stopped ? "🛑 SÍ" : "▶️ NO");
  Serial.print("  PWM Actual: ");
  Serial.println(leftMotor.pwm);
  Serial.println("");
  
  Serial.println("🔧 Estado Motor Derecho:");
  Serial.print("  Habilitado: ");
  Serial.println(rightMotor.enabled ? "✅ SÍ" : "❌ NO");
  Serial.print("  Frenado: ");
  Serial.println(rightMotor.braked ? "🔒 SÍ" : "🔓 NO");
  Serial.print("  Detenido: ");
  Serial.println(rightMotor.stopped ? "🛑 SÍ" : "▶️ NO");
  Serial.print("  PWM Actual: ");
  Serial.println(rightMotor.pwm);
  Serial.println("");
  
  // 📈 SECCIÓN: INFORMACIÓN DEL SISTEMA (EXPANDIBLE)
  Serial.println("📈 INFORMACIÓN DEL SISTEMA:");
  Serial.println("───────────────────────────────────────────────");
  
  Serial.print("⚡ Voltaje del sistema: ");
  #ifdef VOLTAGE_MONITOR_PIN
  int voltageRaw = analogRead(VOLTAGE_MONITOR_PIN);
  float voltage = (voltageRaw * 5.0 / 1023.0) * VOLTAGE_DIVIDER_RATIO;
  Serial.print(voltage);
  Serial.println(" V");
  #else
  Serial.println("No configurado");
  #endif
  
  Serial.print("🕐 Tiempo de funcionamiento: ");
  unsigned long uptime = millis();
  Serial.print(uptime / 1000);
  Serial.println(" segundos");
  
  Serial.print("💾 Memoria libre: ");
  extern int __heap_start, *__brkval;
  int v;
  Serial.print((int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
  Serial.println(" bytes");
  
  Serial.println("");
  Serial.println("===============================================");
  Serial.println("🎯 COMANDOS: SYSTEM_PINS, ALL_PINS, PINS_COMPLETE");
  Serial.println("📝 Esta función se expande automáticamente");
  Serial.println("   cuando agregas nuevos pines al sistema");
  Serial.println("===============================================");
}

/**
 * MOSTRAR ESTADO DE PINES STOP
 */
void showStopPinStatus() {
  Serial.print("STOP_LEFT_MOTOR (pin ");
  Serial.print(STOP_LEFT_MOTOR);
  Serial.print("): ");
  int leftRead = digitalRead(STOP_LEFT_MOTOR);
  Serial.print(leftRead ? "HIGH" : "LOW");
  Serial.println(" (INPUT=FLOAT/ENABLE, OUTPUT LOW=DISABLE)");
  
  Serial.print("STOP_RIGHT_MOTOR (pin ");
  Serial.print(STOP_RIGHT_MOTOR);
  Serial.print("): ");
  int rightRead = digitalRead(STOP_RIGHT_MOTOR);
  Serial.print(rightRead ? "HIGH" : "LOW");
  Serial.println(" (INPUT=FLOAT/ENABLE, OUTPUT LOW=DISABLE)");
}

/**
 * MOSTRAR ESTADO DE PINES BRAKE
 */
void showBrakePinStatus() {
  Serial.print("BRAKE_LEFT_MOTOR (pin ");
  Serial.print(BRAKE_LEFT_MOTOR);
  Serial.print("): ");
  int leftRead = digitalRead(BRAKE_LEFT_MOTOR);
  Serial.print(leftRead ? "HIGH" : "LOW");
  Serial.println(" (OUTPUT HIGH=BRAKE ON, INPUT=FLOAT/BRAKE OFF)");
  
  Serial.print("BRAKE_RIGHT_MOTOR (pin ");
  Serial.print(BRAKE_RIGHT_MOTOR);
  Serial.print("): ");
  int rightRead = digitalRead(BRAKE_RIGHT_MOTOR);
  Serial.print(rightRead ? "HIGH" : "LOW");
  Serial.println(" (OUTPUT HIGH=BRAKE ON, INPUT=FLOAT/BRAKE OFF)");
}

/**
 * FORZAR TODOS los pines de STOP y BRAKE a LOW (OUTPUT LOW)
 */
void forceAllPinsLow() {
  Serial.println("FORZANDO TODOS LOS PINES A OUTPUT LOW:");
  
  // STOP pins a OUTPUT LOW
  pinMode(STOP_LEFT_MOTOR, OUTPUT);
  digitalWrite(STOP_LEFT_MOTOR, LOW);
  Serial.println("STOP_LEFT_MOTOR -> OUTPUT LOW");
  
  pinMode(STOP_RIGHT_MOTOR, OUTPUT);
  digitalWrite(STOP_RIGHT_MOTOR, LOW);
  Serial.println("STOP_RIGHT_MOTOR -> OUTPUT LOW");
  
  // BRAKE pins a OUTPUT LOW
  pinMode(BRAKE_LEFT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_LEFT_MOTOR, LOW);
  Serial.println("BRAKE_LEFT_MOTOR -> OUTPUT LOW");
  
  pinMode(BRAKE_RIGHT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_RIGHT_MOTOR, LOW);
  Serial.println("BRAKE_RIGHT_MOTOR -> OUTPUT LOW");
  
  Serial.println("MIDIENDO VOLTAJES:");
  delay(100);  // Esperar estabilización
  showPinStatus();
}

/**
 * FORZAR TODOS los pines de STOP y BRAKE a HIGH (OUTPUT HIGH)
 */
void forceAllPinsHigh() {
  Serial.println("FORZANDO TODOS LOS PINES A OUTPUT HIGH:");
  
  // STOP pins a OUTPUT HIGH
  pinMode(STOP_LEFT_MOTOR, OUTPUT);
  digitalWrite(STOP_LEFT_MOTOR, HIGH);
  Serial.println("STOP_LEFT_MOTOR -> OUTPUT HIGH");
  
  pinMode(STOP_RIGHT_MOTOR, OUTPUT);
  digitalWrite(STOP_RIGHT_MOTOR, HIGH);
  Serial.println("STOP_RIGHT_MOTOR -> OUTPUT HIGH");
  
  // BRAKE pins a OUTPUT HIGH
  pinMode(BRAKE_LEFT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_LEFT_MOTOR, HIGH);
  Serial.println("BRAKE_LEFT_MOTOR -> OUTPUT HIGH");
  
  pinMode(BRAKE_RIGHT_MOTOR, OUTPUT);
  digitalWrite(BRAKE_RIGHT_MOTOR, HIGH);
  Serial.println("BRAKE_RIGHT_MOTOR -> OUTPUT HIGH");
  
  Serial.println("MIDIENDO VOLTAJES:");
  delay(100);  // Esperar estabilización
  showPinStatus();
}

/**
 * SECUENCIA DIAGNÓSTICA COMPLETA
 */
void diagnosticPinSequence() {
  Serial.println("INICIANDO DIAGNOSTICO COMPLETO DE PINES");
  Serial.println("=======================================");
  
  Serial.println("");
  Serial.println("1- ESTADO INICIAL (sin modificar):");
  showPinStatus();
  
  delay(2000);
  
  Serial.println("");
  Serial.println("2- FORZANDO TODOS A LOW:");
  forceAllPinsLow();
  
  delay(3000);
  
  Serial.println("");
  Serial.println("3- FORZANDO TODOS A HIGH:");
  forceAllPinsHigh();
  
  delay(3000);
  
  Serial.println("");
  Serial.println("4- RESTAURANDO CONFIGURACION CORRECTA:");
  Serial.println("   - STOPs a INPUT (FLOAT/ENABLE)");
  Serial.println("   - BRAKEs a INPUT (FLOAT/INACTIVE)");
  
  // Configuración correcta según especificaciones
  pinMode(STOP_LEFT_MOTOR, INPUT);     // FLOAT = ENABLE
  pinMode(STOP_RIGHT_MOTOR, INPUT);    // FLOAT = ENABLE
  pinMode(BRAKE_LEFT_MOTOR, INPUT);    // FLOAT = BRAKE INACTIVE
  pinMode(BRAKE_RIGHT_MOTOR, INPUT);   // FLOAT = BRAKE INACTIVE
  
  delay(100);
  
  Serial.println("");
  Serial.println("5- ESTADO FINAL (configuracion correcta):");
  showPinStatus();
  
  Serial.println("");
  Serial.println("DIAGNOSTICO COMPLETO TERMINADO");
  Serial.println("Si los voltajes no cambian entre LOW/HIGH,");
  Serial.println("verifica las conexiones fisicas y la alimentacion");
}

#endif // ENABLE_SERIAL_COMMANDS

#endif // SERIAL_COMMAND_PROCESSOR_H
