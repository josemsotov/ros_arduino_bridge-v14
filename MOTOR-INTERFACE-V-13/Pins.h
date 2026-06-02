/**
 * MOTOR-INTERFACE-V-13 PIN DEFINITIONS
 * Smart Golf Trolley - Definiciones de pines del hardware
 * 
 * 🔒 **CONFIGURATION LOCKED BY USER - October 10, 2025** 🔒
 * 
 * ⚠️  CRITICAL WARNING: DO NOT MODIFY PIN ASSIGNMENTS ⚠️
 * ⚠️  USER HAS MANUALLY CONFIGURED THESE PINS FOR SPECIFIC HARDWARE ⚠️
 * ⚠️  CHANGES ONLY ALLOWED WITH EXPLICIT USER REQUEST ⚠️
 * 
 * ADVERTENCIA: NO MODIFICAR ESTOS VALORES SIN VERIFICAR EL HARDWARE
 * Los pines están asignados según el diseño del PCB y drivers utilizados
 * 
 * See PIN_CONFIGURATION_LOCK.md for full documentation
 */

#ifndef PINS_H
#define PINS_H

//===========================================================================
//========================== PINES DE MOTORES ==============================
//===========================================================================

/**
 * DRIVER ZS-X11H - MOTOR IZQUIERDO
 * Cable colors: PWM=Blanco, DIR=Rojo, BRAKE=Blanco, STOP=Azul
 * VERIFICADO 2026-04-24: MB (antes pines 46/52/50/48) es el motor izquierdo físico
 */
#define PWM_LEFT_MOTOR      46    // Pin PWM motor izquierdo   (Cable Blanco)
#define DIR_LEFT_MOTOR      52    // Pin dirección motor izq   (Cable Rojo)
#define BRAKE_LEFT_MOTOR    50    // Pin freno motor izquierdo (Cable Blanco)
#define STOP_LEFT_MOTOR     48    // Pin STOP motor izquierdo  (Cable Azul)


/**
 * DRIVER ZS-X11H - MOTOR DERECHO
 * Cable colors: PWM=Blanco, DIR=Rojo, BRAKE=Blanco, STOP=Azul
 * VERIFICADO 2026-04-24: MA (antes pines 44/30/28/26) es el motor derecho físico
 */
#define PWM_RIGHT_MOTOR     44    // Pin PWM motor derecho     (Cable Blanco)
#define DIR_RIGHT_MOTOR     30    // Pin dirección motor der   (Cable Rojo)
#define BRAKE_RIGHT_MOTOR   28    // Pin freno motor derecho   (Cable Blanco)
#define STOP_RIGHT_MOTOR    26    // Pin STOP motor derecho    (Cable Azul)


//===========================================================================
//========================== PINES DE SENSORES =============================
//===========================================================================

/**
 * SENSORES HALL - RETROALIMENTACIÓN VELOCIDAD
 */
#ifdef ENABLE_HALL_SENSORS
  #define HALL_LEFT_MOTOR   19    // Interrupción 4 - Hall sensor izquierdo
  #define HALL_RIGHT_MOTOR  18    // Interrupción 5 - Hall sensor derecho
#endif

/**
 * OPTOENCODERS - RETROALIMENTACIÓN POSICIÓN  
 */
#ifdef ENABLE_OPTO_ENCODERS
  #define OPTO_LEFT_MOTOR   20    // Interrupción 3 - OptoEncoder izquierdo
  #define OPTO_RIGHT_MOTOR  21    // Interrupción 2 - OptoEncoder derecho
#endif

/**
 * SENSOR INERCIAL MPU9250/6500 - I2C
 * IMU de 6 o 9 ejes (acelerómetro + giroscopio + magnetómetro)
 */
#ifdef ENABLE_MPU9250
  #define MPU_SDA_PIN       20    // Pin SDA (I2C Data)  (Cable Amarillo)
  #define MPU_SCL_PIN       21    // Pin SCL (I2C Clock) (Cable Verde)
  // VCC = 3.3V (Cable Rojo), GND (Cable Negro)
  // Nota: En Arduino Mega, SDA=20 y SCL=21 son los pines I2C por defecto
  // El MPU se comunica por I2C en dirección 0x68
#endif

//===========================================================================
//========================== PINES DE CONTROL ==============================
//===========================================================================

/**
 * JOYSTICK ANALÓGICO - CONTROL MANUAL
 * Joystick de 2 ejes + botón para control directo del robot
 */
#ifdef ENABLE_JOYSTICK
  #define JOYSTICK_SW_PIN   A0    // Pin switch/botón del joystick
  #define JOYSTICK_Y_PIN    A1    // Pin eje Y (giro/delta velocidades)
  #define JOYSTICK_X_PIN    A2    // Pin eje X (velocidad/dirección)
  // Nota: Joystick usa entradas analógicas (0-1023)
  // Botón usa lógica inversa con pull-up (LOW = presionado)
#endif


/**
 * BLUETOOTH HC-05 - Serial2 (TX2=16 / RX2=17)
 * ENABLE (pin AT mode) y STATE (indicador de conexión)
 */
#ifdef ENABLE_BLUETOOTH
  #define BT_SERIAL         Serial2   // TX2=pin16, RX2=pin17
  #define BT_BAUD_RATE      9600      // Velocidad por defecto HC-05
  #define BT_ENABLE_PIN     40        // HIGH = modo AT, LOW = modo datos
  #define BT_STATE_PIN      42        // HIGH = conectado, LOW = sin par
#endif

/**
 * GPS - Serial3 (TX3=14 / RX3=15)
 */
#ifdef ENABLE_GPS
  #define GPS_SERIAL        Serial3   // TX3=pin14, RX3=pin15
  #define GPS_BAUD_RATE     9600      // Velocidad estándar NMEA
#endif

//===========================================================================
//====================== PINES AUXILIARES Y DEBUG ==========================
//===========================================================================

/**
 * LEDS DE ESTADO (OPCIONAL)
 */
//#define LED_STATUS_PIN    13    // LED indicador estado general
//#define LED_ERROR_PIN     11    // LED indicador error/alarma

/**
 * PINES LIBRES DISPONIBLES
 * Pines que pueden usarse para expansiones futuras
 */
/*
  PINES DIGITALES LIBRES: 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 
                          22, 23, 24, 25, 27, 29, 31, 32, 33, 34, 35, 
                          36, 37, 38, 39, 40, 41, 42, 43, 45, 47, 48, 49, 51
  
  PINES ANALÓGICOS LIBRES: A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15
  
  INTERRUPCIONES LIBRES: 0(pin 2), 1(pin 3)
*/

//===========================================================================
//======================= MAPEO DE INTERRUPCIONES =========================
//===========================================================================

/**
 * ARDUINO MEGA 2560 - MAPEO INTERRUPCIONES
 */
#ifdef ENABLE_HALL_SENSORS
  #define HALL_LEFT_INTERRUPT   4   // digitalPinToInterrupt(19)
  #define HALL_RIGHT_INTERRUPT  5   // digitalPinToInterrupt(18)
#endif

#ifdef ENABLE_OPTO_ENCODERS  
  #define OPTO_LEFT_INTERRUPT   3   // digitalPinToInterrupt(20)
  #define OPTO_RIGHT_INTERRUPT  2   // digitalPinToInterrupt(21)
#endif

//===========================================================================
//==================== VALIDACIÓN DE PINES ===============================
//===========================================================================

/**
 * VERIFICACIONES DE SEGURIDAD
 * Evitar conflictos de pines entre diferentes funcionalidades
 */



// Verificar que no se usen pines reservados del sistema
#if defined(PWM_LEFT_MOTOR) && (PWM_LEFT_MOTOR < 2)
  #error "PWM_LEFT_MOTOR no puede usar pines 0-1 (reservados para Serial)"
#endif

#if defined(PWM_RIGHT_MOTOR) && (PWM_RIGHT_MOTOR < 2)  
  #error "PWM_RIGHT_MOTOR no puede usar pines 0-1 (reservados para Serial)"
#endif

#endif // PINS_H