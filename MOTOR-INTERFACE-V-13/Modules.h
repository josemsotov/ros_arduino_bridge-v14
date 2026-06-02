/**
 * MOTOR-INTERFACE-V-13 FUNCTIONAL MODULES
 * Smart Golf Trolley - Inclusión condicional de módulos funcionales
 * 
 * Este archivo incluye los módulos necesarios basándose en la configuración
 * Estilo Marlin: Solo se compilan las funcionalidades habilitadas
 */

#ifndef MODULES_H
#define MODULES_H

// Incluir configuración principal
#include "Configuration.h"
#include "Pins.h"

//===========================================================================
//======================= INCLUSIÓN DE MÓDULOS CORE =======================
//===========================================================================

/**
 * MÓDULOS BÁSICOS - SIEMPRE INCLUIDOS
 * NOTA: El orden es importante para las dependencias
 */
#include "Motor_Control.h"       // Control básico de motores (primero - define MotorState)
#include "Robot_States.h"        // Sistema de estados del robot (segundo - usa MotorState)

//===========================================================================
//==================== INCLUSIÓN CONDICIONAL DE MÓDULOS ===================
//===========================================================================

/**
 * SISTEMAS DE CONTROL AVANZADOS ELIMINADOS
 * Solo control básico de motores disponible
 */

/**
 * SENSORES
 */
#ifdef ENABLE_HALL_SENSORS
  #include "Hall_Sensors.h"      // Manejo sensores Hall (ISRs con signo)
#endif

/**
 * SENSOR INERCIAL MPU9250/6500
 * IMPORTANTE: Incluir ANTES de Odometry (que llama mpu_getGyroZ) y Core_Functions
 */
#ifdef ENABLE_MPU9250
  #include "MPU9250.h"           // Sensor inercial IMU (DLPF + EMA)
#endif

/**
 * ODOMETRÍA
 * IMPORTANTE: Incluir DESPUÉS de Hall_Sensors y MPU9250, ANTES de ROS2_Bridge
 */
#ifdef ENABLE_ODOMETRY
  #ifdef ENABLE_HALL_SENSORS
    #include "Odometry.h"        // Fusión Hall + MPU9250 (filtro complementario)
  #endif
#endif

/**
 * CONTROL PID
 * Disponible siempre para ROS2 y comandos manuales
 */
#include "PID_Control.h"

/**
 * BLUETOOTH HC-05
 * IMPORTANTE: Incluir ANTES de ROS2_Bridge (usa bt_connected) y Core_Functions (usa bt_initialize)
 */
#ifdef ENABLE_BLUETOOTH
  #include "Bluetooth_Module.h"    // Comunicación inalámbrica HC-05
#endif

/**
 * ROS2 BRIDGE
 * IMPORTANTE: Incluir después de sensores y Bluetooth, antes de joystick
 */
#ifdef ENABLE_ROS2_BRIDGE
  #include "ROS2_Bridge.h"       // Comunicación con Raspberry Pi 5
#endif

/**
 * JOYSTICK DE CONTROL
 * IMPORTANTE: Debe incluirse ANTES de Core_Functions.h
 */
#ifdef ENABLE_JOYSTICK
  #include "Joystick_Module.h"   // Control por joystick analógico
#endif

/**
 * FUNCIONES CORE DEL SISTEMA
 * IMPORTANTE: Debe incluirse DESPUÉS de los módulos que usa (MPU, Joystick, Odometry, BT)
 */
#include "Core_Functions.h"      // Funciones básicas del sistema

/**
 * GPS
 * IMPORTANTE: Incluir después de Core_Functions
 */
#ifdef ENABLE_GPS
  #include "GPS_Module.h"          // Módulo GPS NMEA
#endif

/**
 * COMUNICACIÓN SERIAL
 */
#ifdef ENABLE_SERIAL_COMMANDS
  #include "Serial_Command_Processor.h"   // Procesador de comandos serie
#endif
//===========================================================================
//====================== VERIFICACIÓN DE MÓDULOS ==========================
//===========================================================================

/**
 * CONTADORES DE FUNCIONALIDADES ACTIVAS
 * Para verificar que la configuración es coherente
 */

// Contar módulos de control activos
#define CONTROL_MODULES_COUNT 0

// Contar tipos de sensores activos  
#define SENSOR_MODULES_COUNT \
  (defined(ENABLE_HALL_SENSORS) ? 1 : 0) + \
  (defined(ENABLE_OPTO_ENCODERS) ? 1 : 0)

// Verificaciones mínimas
#if SENSOR_MODULES_COUNT == 0
  #error "Debe habilitar al menos un tipo de sensor"
#endif

#if CONTROL_MODULES_COUNT == 0
  #warning "No hay módulos de control habilitados - funcionalidad limitada"
#endif

//===========================================================================
//==================== INFORMACIÓN DE COMPILACIÓN ========================
//===========================================================================

/**
 * INFORMACIÓN PARA EL COMPILADOR
 * Se muestra durante la compilación para verificar configuración
 */

#ifdef ENABLE_VERBOSE_OUTPUT
  #pragma message "CONFIGURACIÓN MOTOR-INTERFACE-V-13:"
  
  #ifdef ENABLE_DUAL_CROSSING_CONTROL
    #pragma message "  ✓ Control Dual de Cruce habilitado"
  #endif
  
  #ifdef ENABLE_PID_CONTROL
    #pragma message "  ✓ Control PID habilitado"  
  #endif

  #ifdef ENABLE_DUAL_CROSSING_CONTROL
    #pragma message "  ✓ Control Dual de Cruce habilitado"
  #endif

  #ifdef ENABLE_HALL_SENSORS
    #pragma message "  ✓ Sensores Hall habilitados"
  #endif
  
  #ifdef ENABLE_OPTO_ENCODERS
    #pragma message "  ✓ OptoEncoders habilitados"
  #endif
#endif

#endif // MODULES_H