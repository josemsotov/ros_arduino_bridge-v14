/**
 * MOTOR-INTERFACE-V-13 CONFIGURATION
 * Smart Golf Trolley - Archivo de configuración principal
 * Estilo Marlin Firmware - Activar/Desactivar funcionalidades con #define
 * 
 * INSTRUCCIONES:
 * - Comentar (//) una línea #define para DESACTIVAR la funcionalidad
 * - Descomentar una línea #define para ACTIVAR la funcionalidad
 * - Solo modificar los valores después de verificar compatibilidad
 */

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//===========================================================================
//========================== FUNCIONALIDADES PRINCIPALES ==================
//===========================================================================

/**
 * SISTEMA DE CONTROL BÁSICO
 * Solo control directo de motores disponible
 */
//#define ENABLE_DUAL_CROSSING_CONTROL    // Sistema dual eliminado completamente

/**
 * SISTEMA PID
 * Control automático de velocidad con retroalimentación
 * NOTA: Si PID_CONTROL está desactivado, todas las opciones PID se desactivan automáticamente
 */
//#define ENABLE_PID_CONTROL             // Activar control PID

#ifdef ENABLE_PID_CONTROL
  #define ENABLE_PID_TUNING           // Permitir ajuste de parámetros PID vía comandos
  #define ENABLE_PID_DEBUG            // Debug información PID
#endif

/**
 * SENSORES
 * Habilitar diferentes tipos de sensores de feedback
 * NOTA: DUAL_SENSOR_MODE solo se activa si ambos sensores están habilitados
 */
#define ENABLE_HALL_SENSORS           // Activar sensores Hall
//#define ENABLE_OPTO_ENCODERS          // Activar OptoEncoders

#if defined(ENABLE_HALL_SENSORS) && defined(ENABLE_OPTO_ENCODERS)
  #define ENABLE_DUAL_SENSOR_MODE     // Usar ambos sensores simultáneamente
#endif

/**
 * SENSOR INERCIAL MPU9250/6500
 * IMU de 6 o 9 ejes con giroscopio y acelerómetro
 * CARACTERÍSTICAS:
 * - Filtrado DLPF Hardware + EMA Software
 * - Sample Rate: 200 Hz
 * - Filtro complementario pitch (accel + gyro Y)
 * - Integración de ángulos yaw
 * NOTA: Requiere conexión I2C (SDA=pin20, SCL=pin21)
 */
#define ENABLE_MPU9250                // Activar sensor MPU9250/6500

#ifdef ENABLE_MPU9250
  //#define ENABLE_MPU_DEBUG          // Debug información MPU (DESACTIVADO para ahorrar RAM)
  #define ENABLE_MPU_AUTO_CALIBRATION // Calibración automática al inicio
#endif

/**
 * BALANCE ANTI-CAÍDA — Control de equilibrio dinámico del robot
 * Detecta la inclinación del chasis (MPU9250) y añade una corrección PD
 * a la velocidad del usuario para evitar que el robot se caiga:
 *   • Chasis se inclina adelante → corrección positiva → ruedas aceleran
 *   • Chasis se inclina atrás    → corrección negativa → ruedas frenan
 *
 * Ley de control: correction = Kp * pitch_error + Kd * gyroY
 * Se activa automáticamente al arrancar. Solo para diagnóstico desactivar.
 * REQUIERE: ENABLE_MPU9250 activo
 * COMANDOS: hb on | hb off | hb cal | hb stat | hb kp <v> | hb kd <v>
 */
#define ENABLE_HOVERBOARD_MODE        // Activar balance anti-caída

#ifdef ENABLE_HOVERBOARD_MODE
  // Ganancia proporcional: corrección m/s por grado de inclinación
  // Subir si el robot sigue cayendo; bajar si oscila
  #define HOVERBOARD_KP               0.025f  // m/s / °
  // Ganancia derivativa: amortigua oscilaciones (usa gyroY directamente)
  // Subir si hay oscilaciones; bajar si hay sobreactuación
  #define HOVERBOARD_KD               0.002f  // m/s / (°/s)
  // Zona muerta: inclinaciones menores a este ángulo no generan corrección
  #define HOVERBOARD_DEAD             1.5f    // ° — sin corrección bajo este umbral

  // ── LÍMITES DE INCLINACIÓN ASIMÉTRICOS ─────────────────────────────────
  // Límite FRONTAL: error de pitch positivo (robot cae hacia adelante)
  // Si pitch_error > FRONT_LIMIT → caída detectada → emergency stop
  // Calibrar con: 'hb cal front' (robot inclinado al límite seguro adelante)
  #define HOVERBOARD_FRONT_LIMIT_DEG  35.0f   // °
  // Límite TRASERO: error de pitch negativo (robot cae hacia atrás)
  // Si pitch_error < -REAR_LIMIT → caída detectada → emergency stop
  // Calibrar con: 'hb cal rear'  (robot inclinado al límite seguro atrás)
  #define HOVERBOARD_REAR_LIMIT_DEG   35.0f   // °

  // Alpha del filtro complementario del MPU (0.98 = 98% gyro + 2% accel)
  #define HOVERBOARD_COMP_ALPHA       0.98f
  // Signo de pitch: +1.0 si adelante = pitch positivo
  // Cambiar a -1.0 si el robot va al revés
  #define HOVERBOARD_PITCH_SIGN       1.0f
#endif

/**
 * SENSORES DE CORRIENTE ACS712
 * Monitoreo en tiempo real de la corriente consumida por cada motor.
 * Motor DERECHO  → ACS712 → A3
 * Motor IZQUIERDO → ACS712 → A4
 *
 * MODELOS (seleccionar sensibilidad):
 *   5A  → ACS712_SENSITIVITY_VA 0.185f
 *   20A → ACS712_SENSITIVITY_VA 0.100f  ← más común para motores DC
 *   30A → ACS712_SENSITIVITY_VA 0.066f
 */
// #define ENABLE_CURRENT_SENSORS   // ACS712 desconectado del circuito de potencia

#ifdef ENABLE_CURRENT_SENSORS
  // Sensibilidad del modelo ACS712 en V/A
  // Cambia este valor según el modelo que tengas soldado:
  //   5A  → 0.185f  |  20A → 0.100f  |  30A → 0.066f
  #define ACS712_SENSITIVITY_VA   0.100f    // ACS712-20A por defecto
  // Factor de suavizado EMA (0.0-1.0). Más alto = más rápido pero más ruido
  #define ACS712_FILTER_ALPHA     0.15f
#endif

/**
 * ROS2 BRIDGE - COMUNICACIÓN CON RASPBERRY PI 5
 * Bridge de comunicación serial compatible con ROS2
 * CARACTERÍSTICAS:
 * - Protocolo de comandos de velocidad (cmd_vel)
 * - Publicación de odometría en tiempo real
 * - Lectura de encoders para navegación
 * - Safety timeout automático
 * - Compatible con nav2 stack
 * NOTA: Usa puerto Serial principal (115200 baud)
 */
#define ENABLE_ROS2_BRIDGE              // Activar bridge ROS2

#ifdef ENABLE_ROS2_BRIDGE
  #define ROS2_AUTO_START             // Auto-iniciar bridge al boot
  //#define ROS2_DEBUG                // Debug información ROS2 (DESACTIVADO en producción)
#endif

/**
 * JOYSTICK DE CONTROL
 * Joystick analógico de 2 ejes + botón
 * CARACTERÍSTICAS:
 * - Eje X: Velocidad/Dirección (adelante/atrás)
 * - Eje Y: Giro (control diferencial)
 * - Botón: Activar modo tanque
 * - Zona muerta configurable
 * - Anti-rebote en botón
 * NOTA: Requiere 3 pines analógicos/digitales
 */
#define ENABLE_JOYSTICK               // Activar control por joystick

#ifdef ENABLE_JOYSTICK
  //#define ENABLE_JOYSTICK_DEBUG     // Debug información joystick (DESACTIVADO para ahorrar RAM)
  #define ENABLE_JOYSTICK_TANK_MODE   // Habilitar modo tanque
#endif

/**
 * COMUNICACIÓN SERIAL
 * Comandos de depuración y control
 * NOTA: Las opciones SERIAL se desactivan automáticamente si SERIAL_COMMANDS está desactivado
 * NOTA: VERBOSE_OUTPUT desactivado para ahorrar RAM (puede reactivarse si es necesario)
 */
#define ENABLE_SERIAL_COMMANDS        // Activar comandos por puerto serie

#ifdef ENABLE_SERIAL_COMMANDS
  //#define ENABLE_VERBOSE_OUTPUT     // Salida detallada de información (DESACTIVADO para ahorrar RAM)
  #define ENABLE_COMMAND_HELP         // Sistema de ayuda de comandos
#endif

/**
 * SISTEMA DE CALIBRACIÓN
 * Auto-calibración de PPR (DESHABILITADO - PPR FIJOS)
 */
//#define ENABLE_PPR_CALIBRATION      // PPR ya están definidos como constantes

/**
 * FUNCIONES DE PRUEBA Y DEBUG
 * Herramientas de desarrollo y diagnóstico
 */
#define ENABLE_MOTOR_TESTS            // Funciones de prueba de motores
#define ENABLE_HARDWARE_TESTS         // Pruebas de hardware (sensores, pines)
#define ENABLE_SYSTEM_STATUS          // Información de estado del sistema

//===========================================================================
//======================= CONFIGURACIONES AVANZADAS =======================
//===========================================================================

/**
 * CONFIGURACIÓN PID
 */
#ifdef ENABLE_PID_CONTROL
  #define DEFAULT_KP_LEFT       1.0   // Ganancia proporcional motor izquierdo
  #define DEFAULT_KI_LEFT       0.1   // Ganancia integral motor izquierdo  
  #define DEFAULT_KD_LEFT       0.05  // Ganancia derivativa motor izquierdo
  
  #define DEFAULT_KP_RIGHT      1.0   // Ganancia proporcional motor derecho
  #define DEFAULT_KI_RIGHT      0.1   // Ganancia integral motor derecho
  #define DEFAULT_KD_RIGHT      0.05  // Ganancia derivativa motor derecho
  
  #define PID_UPDATE_RATE       50    // Frecuencia actualización PID (ms)
#endif



/**
 * CONFIGURACIÓN DIFERENCIAL
 */
#ifdef ENABLE_DIFFERENTIAL_CONTROL
  #define DIFFERENTIAL_MIN_PWM  10    // PWM mínimo para giros
  #define DIFFERENTIAL_MAX_PWM  40    // PWM máximo para giros
  #define DIFFERENTIAL_TIMEOUT  5000  // Timeout giros automáticos (ms)
#endif

/**
 * CONFIGURACIÓN SENSORES
 */
#ifdef ENABLE_HALL_SENSORS
  #define PPR_HALL_SENSORS      45    // Pulsos por revolución sensores Hall
#endif

#ifdef ENABLE_OPTO_ENCODERS  
  #define PPR_OPTO_ENCODERS     40    // Pulsos por revolución OptoEncoders
#endif

/**
 * CONFIGURACIÓN COMUNICACIÓN SERIAL
 */
#ifdef ENABLE_SERIAL_COMMANDS
  #define SERIAL_BAUD_RATE      115200  // Velocidad puerto serie
  #define COMMAND_BUFFER_SIZE   50      // Tamaño buffer comandos
  #define RESPONSE_DELAY        10      // Delay entre respuestas (ms)
#endif

//===========================================================================
//========================= MACROS DE DEBUG ================================
//===========================================================================

/**
 * MACROS DE DEBUG CONDICIONAL
 * Estas macros deben estar disponibles en todos los módulos
 * Soportan argumentos variables para formateo (ej: DEBUG_PRINT(valor, 2) para 2 decimales)
 */
#ifdef ENABLE_VERBOSE_OUTPUT
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

// Macro para comandos de ayuda condicionales
#ifdef ENABLE_COMMAND_HELP
  #define HELP_COMMAND(cmd, desc) printHelpCommand(cmd, desc)
#else
  #define HELP_COMMAND(cmd, desc)
#endif

//===========================================================================
//========================= VALIDACIÓN DE CONFIGURACIÓN ====================
//===========================================================================

// Verificación de dependencias
#ifdef ENABLE_DIFFERENTIAL_CONTROL
  #ifndef ENABLE_DUAL_CROSSING_CONTROL
    #error "DIFFERENTIAL_CONTROL requiere DUAL_CROSSING_CONTROL activado" 
  #endif
#endif

// Validación PID eliminada - ahora es automática mediante #ifdef ENABLE_PID_CONTROL



// Verificación de sensores
#if !defined(ENABLE_HALL_SENSORS) && !defined(ENABLE_OPTO_ENCODERS)
  #error "Debe activar al menos un tipo de sensor (HALL o OPTO)"
#endif

#endif // CONFIGURATION_H