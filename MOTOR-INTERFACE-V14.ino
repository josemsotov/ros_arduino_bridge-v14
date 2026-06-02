/**
 * MOTOR-INTERFACE-V-13 MARLIN STYLE
 * Smart Golf Trolley - Archivo principal estilo Marlin Firmware
 * 
 * INSTRUCCIONES DE USO:
 * 1. Modifica Configuration.h para activar/desactivar funcionalidades
 * 2. Ajusta Pins.h solo si cambias el hardware
 * 3. Este archivo no necesita modificaciones
 * 
 * ESTRUCTURA MODULAR:
 * - Configuration.h: Activar/desactivar funcionalidades
 * - Pins.h: Definiciones de pines del hardware
 * - Modules.h: Inclusión condicional de módulos
 * - [Módulo]_[Función].h: Funcionalidades específicas
 * 
 * COMPILACIÓN:
 * Solo se compilan los módulos habilitados en Configuration.h
 */

// ============================================================================
// ========================== INCLUDES PRINCIPALES ==========================
// ============================================================================

// Inclusión básica de Arduino (necesaria para algunas configuraciones)
#include <Arduino.h>

// Librería I2C para comunicación con sensores (MPU9250, etc.)
#include <Wire.h>

// Configuración del sistema (MODIFICAR ESTE ARCHIVO PARA PERSONALIZAR)
#include "Configuration.h"

// Definiciones de pines del hardware
#include "Pins.h"  

// Inclusión condicional de módulos basada en configuración
#include "Modules.h"

// ============================================================================
// ========================== DECLARACIONES DE FUNCIONES ====================
// ============================================================================

/**
 * FUNCIONES DUMMY PARA MÓDULOS DESHABILITADOS
 * Estas funciones se declaran antes del setup para evitar errores de compilación
 */



#ifndef ENABLE_OPTO_ENCODERS
void updateOptoSpeeds() { 
  // Función vacía - OptoEncoders deshabilitados
}
#endif

#ifndef ENABLE_PID_CONTROL
void updatePIDControl() { 
  // Función vacía - Control PID deshabilitado
}
#endif

// Funciones de control avanzado eliminadas

#ifndef ENABLE_HALL_SENSORS
void updateHallSpeeds() {
  // Función vacía - Sensores Hall deshabilitados
}
#endif

#ifndef ENABLE_SERIAL_COMMANDS
void readSerialCommands() {
  // Función vacía - Comandos Serie deshabilitados
}
void processSerialCommand() {
  // Función vacía - Comandos Serie deshabilitados
}
void processContinuousDebug() {
  // Función vacía - Comandos Serie deshabilitados
}
#endif

// ============================================================================
// ========================== SETUP PRINCIPAL ================================
// ============================================================================

void setup() {
  // Inicializar comunicacion serie
  #ifdef ENABLE_SERIAL_COMMANDS
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 5000);  // Esperar conexion (max 5s)
    delay(100);
  #endif
  
  // Mensaje de bienvenida
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("===============================================");
  DEBUG_PRINTLN("   MOTOR-INTERFACE-V-13 MARLIN STYLE");
  DEBUG_PRINTLN("   Smart Golf Trolley - Sistema Modular");
  DEBUG_PRINTLN("===============================================");
  DEBUG_PRINTLN("");
  
  // Mostrar configuracion activa
  #ifdef ENABLE_VERBOSE_OUTPUT
    DEBUG_PRINTLN("Configuracion cargada:");
    DEBUG_PRINTLN("  [OK] Control Básico de Motores");
    #ifdef ENABLE_PID_CONTROL
      DEBUG_PRINTLN("  [OK] Control PID");
    #endif
    #ifdef ENABLE_HALL_SENSORS
      DEBUG_PRINTLN("  [OK] Sensores Hall");
    #endif
    #ifdef ENABLE_OPTO_ENCODERS
      DEBUG_PRINTLN("  [OK] OptoEncoders");
    #endif

    DEBUG_PRINTLN("");
  #endif
  
  // Inicializar sistema principal
  initializeSystem();
  
  // Inicializar ROS2 Bridge
  #ifdef ENABLE_ROS2_BRIDGE
    ros2_initialize();
  #endif
  
  // Mensaje final
  DEBUG_PRINTLN("✓ Sistema inicializado correctamente");
  DEBUG_PRINTLN("✓ Listo para recibir comandos");
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Usa 'HELP' para ver comandos disponibles");
  DEBUG_PRINTLN("===============================================");
  DEBUG_PRINTLN("");
}

// ============================================================================================================================================================================================================
// ========================== LOOP PRINCIPAL =================================================================================================================================================
// ============================================================================================================================================================================================================================

void loop() {
  // ===== PROCESAMIENTO DE COMANDOS SERIE =====
  #ifdef ENABLE_SERIAL_COMMANDS
    readSerialCommands();
    processSerialCommand();
    processContinuousDebug();
  #endif
  
  // ===== ACTUALIZACIÓN MPU + HOVERBOARD =====
  #ifdef ENABLE_MPU9250
    mpu_update();                 // Leer IMU y actualizar pitch/yaw (200 Hz, rate-limited interno)
  #endif
  #if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)
    hoverboard_update();          // Calcular setpoints desde pitch e inyectarlos en ROS2 (50 Hz)
  #endif

  // ===== ROS2 BRIDGE UPDATE =====
  #ifdef ENABLE_ROS2_BRIDGE
    ros2_update();  // Procesar comandos ROS2 y publicar datos
  #endif
  
  // ===== ACTUALIZACIÓN DE ESTADOS DEL ROBOT =====
  updateCrossing();               // Actualizar cruces en progreso
  updatePruebaTotal();            // Actualizar prueba total si está activa
  
  // ===== ACTUALIZACIÓN DE SENSORES =====
  #ifdef ENABLE_HALL_SENSORS
    updateHallSpeeds();           // Calcular velocidades Hall
  #endif
  
  #ifdef ENABLE_OPTO_ENCODERS
    updateOptoSpeeds();           // Calcular velocidades Opto
  #endif
  
  // ===== ACTUALIZACIÓN JOYSTICK =====
  #ifdef ENABLE_JOYSTICK
    updateJoystick();             // Actualizar estado del joystick
  #endif
  
  // ===== PROCESAMIENTO PID =====
  #ifdef ENABLE_PID_CONTROL
    updatePIDControl();           // Actualizar control PID
  #endif
  
  // ===== MONITOREO DEL SISTEMA =====
  systemMonitoring();             // Verificaciones de seguridad
  motorSafetyCheck();             // Verificar seguridad motores
  
  // ===== DELAY MÍNIMO =====
  delay(1);  // Pequeño delay para estabilidad
}

// ============================================================================
// ========================== FUNCIONES AUXILIARES ==========================
// ============================================================================

/**
 * Las funciones auxiliares y dummy ya están declaradas arriba
 * en la sección de declaraciones de funciones
 */

// ============================================================================
// ========================== INFORMACIÓN DE COMPILACIÓN ===================
// ============================================================================

/**
 * Esta información se muestra durante la compilación
 * para verificar qué módulos están siendo incluidos
 */

#ifdef ENABLE_VERBOSE_OUTPUT
  #pragma message "✓ Compilando con CONTROL BÁSICO DE MOTORES únicamente"
  
  #if defined(ENABLE_HALL_SENSORS) && defined(ENABLE_OPTO_ENCODERS)
    #pragma message "✓ Compilando con SENSORES DUALES (Hall + Opto)"
  #elif defined(ENABLE_HALL_SENSORS)
    #pragma message "✓ Compilando con SENSORES HALL únicamente"
  #elif defined(ENABLE_OPTO_ENCODERS)
    #pragma message "✓ Compilando con OPTOENCODERS únicamente"
  #endif
#endif

/**
 * CONFIGURACIÓN DE MEMORIA
 * Información sobre uso de memoria según módulos activos
 */
/*#pragma message ""
#pragma message "MOTOR-INTERFACE-V-13 MARLIN STYLE - Compilación iniciada"
#pragma message "Revisa Configuration.h para modificar funcionalidades"
#pragma message ""
*/