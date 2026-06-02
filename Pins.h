/**
 * MOTOR-INTERFACE-V-13 PIN DEFINITIONS
 * Smart Golf Trolley - Definiciones de pines del hardware
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║          ⚠️  NO CAMBIAR ESTOS PINES — HARDWARE FÍSICO FIJO  ⚠️          ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║                                                                          ║
 * ║  Los pines aquí definidos corresponden al cableado SOLDADO/CONECTADO    ║
 * ║  físicamente en el robot. Cambiarlos en software sin mover los cables   ║
 * ║  causará que los motores NO respondan o se dañe el driver.              ║
 * ║                                                                          ║
 * ║  VERIFICADO Y FUNCIONANDO: 2026-06-02                                   ║
 * ║                                                                          ║
 * ║  Motor IZQUIERDO → PWM=46 (Timer5 ChA), DIR=52, BRAKE=50, STOP=48      ║
 * ║  Motor DERECHO   → PWM=44 (Timer5 ChC*), DIR=30, BRAKE=28, STOP=26     ║
 * ║  Hall IZQUIERDO  → pin 18 (INT5)                                        ║
 * ║  Hall DERECHO    → pin 19 (INT4)                                        ║
 * ║                                                                          ║
 * ║  * Pin 44 usa OCR5C directo via motor_pwm_write() — NO usar             ║
 * ║    analogWrite(44, x) porque limpia los bits COM5C1 del timer.          ║
 * ║                                                                          ║
 * ║  ❌ PROHIBIDO cambiar estos valores sin mover físicamente los cables.   ║
 * ║  ❌ NO usar analogWrite() sobre pines 44/45/46 fuera de Motor_Control.h ║
 * ║                                                                          ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */

#ifndef PINS_H
#define PINS_H

//===========================================================================
//========================== PINES DE MOTORES ==============================
//===========================================================================

/**
 * DRIVER ZS-X11H - MOTOR IZQUIERDO
 * ✅ Cableado verificado y funcionando 2026-06-02
 * CABLE PWM izquierdo → pin 46 del Arduino Mega
 */
#define PWM_LEFT_MOTOR      46    // Timer5 ChA — ✅ verificado 2026-06-02
#define DIR_LEFT_MOTOR      52    // Dirección motor izquierdo
#define BRAKE_LEFT_MOTOR    50    // Freno motor izquierdo (HIGH=freno, FLOAT=libre)
#define STOP_LEFT_MOTOR     48    // STOP driver izquierdo (LOW=disable, FLOAT=enable)


/**
 * DRIVER ZS-X11H - MOTOR DERECHO
 * ✅ Cableado verificado y funcionando 2026-06-02
 * CABLE PWM derecho → pin 44 del Arduino Mega
 * NOTA: usa OCR5C directo via motor_pwm_write() — jamás analogWrite(44,...)
 */
#define PWM_RIGHT_MOTOR     44    // Timer5 ChC via OCR5C — ✅ verificado 2026-06-02
#define DIR_RIGHT_MOTOR     30    // Dirección motor derecho
#define BRAKE_RIGHT_MOTOR   28    // Freno motor derecho (HIGH=freno, FLOAT=libre)
#define STOP_RIGHT_MOTOR    26    // STOP driver derecho (LOW=disable, FLOAT=enable)


//===========================================================================
//========================== PINES DE SENSORES =============================
//===========================================================================

/**
 * SENSORES HALL - RETROALIMENTACIÓN VELOCIDAD
 */
#ifdef ENABLE_HALL_SENSORS
  #define HALL_LEFT_MOTOR   18    // INT5 — Hall motor izquierdo (pin46) — ✅ verificado 2026-06-02
  #define HALL_RIGHT_MOTOR  19    // INT4 — Hall motor derecho  (pin44) — ✅ verificado 2026-06-02
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
  #define MPU_SDA_PIN       20    // Pin SDA (I2C Data) - Arduino Mega default
  #define MPU_SCL_PIN       21    // Pin SCL (I2C Clock) - Arduino Mega default
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
  #define HALL_LEFT_INTERRUPT   5   // digitalPinToInterrupt(18)
  #define HALL_RIGHT_INTERRUPT  4   // digitalPinToInterrupt(19)
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