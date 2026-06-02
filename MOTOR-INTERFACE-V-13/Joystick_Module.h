/**
 * JOYSTICK MODULE - MOTOR-INTERFACE-V-13
 * Módulo de control por joystick analógico
 * 
 * INTEGRACIÓN: 100% del código original preservado
 * PREFIJO: joy_ para todas las funciones y variables
 * COMPILACIÓN CONDICIONAL: Solo si ENABLE_JOYSTICK está definido
 * 
 * HARDWARE:
 * - Joystick analógico de 2 ejes + botón
 * - Eje X: Velocidad/Dirección (adelante/atrás)
 * - Eje Y: Giro (izquierda/derecha) - Control tanque
 * - Switch: Activar/desactivar modo tanque
 * 
 * CARACTERÍSTICAS:
 * - Zona muerta configurable para estabilidad
 * - Modo tanque (control diferencial)
 * - Detección de botón con anti-rebote
 * - Monitoreo en tiempo real
 * 
 * AUTOR: JMS 2025
 * FECHA: Octubre 2025
 * VERSIÓN: 1.0
 */

#ifndef JOYSTICK_MODULE_H
#define JOYSTICK_MODULE_H

#include <Arduino.h>

// ============================================================================
// ======================== CONFIGURACIÓN DEL MÓDULO ==========================
// ============================================================================

/**
 * PINES DEL JOYSTICK
 * Definidos en Pins.h, pero se verifican aquí
 */
#ifndef JOYSTICK_SW_PIN
  #error "JOYSTICK_SW_PIN no está definido en Pins.h"
#endif

#ifndef JOYSTICK_Y_PIN
  #error "JOYSTICK_Y_PIN no está definido en Pins.h"
#endif

#ifndef JOYSTICK_X_PIN
  #error "JOYSTICK_X_PIN no está definido en Pins.h"
#endif

/**
 * PARÁMETROS DE CONFIGURACIÓN
 */
#define JOY_DEADZONE          50      // Zona muerta en el centro (±50 de 512)
#define JOY_CENTER_VALUE      512     // Valor central del ADC (0-1023)
#define JOY_MAX_VALUE         1023    // Valor máximo del ADC
#define JOY_DEBOUNCE_DELAY    300     // Anti-rebote del botón (ms)
#define JOY_UPDATE_INTERVAL   10      // Intervalo de actualización (ms)

// ============================================================================
// ====================== VARIABLES INTERNAS DEL MÓDULO =======================
// ============================================================================

/**
 * VARIABLES GLOBALES DEL JOYSTICK (prefijo joy_)
 * Todas privadas al módulo, accesibles solo mediante getters
 */

// Valores RAW del joystick (0-1023)
static int joy_raw_x = JOY_CENTER_VALUE;
static int joy_raw_y = JOY_CENTER_VALUE;
static int joy_raw_sw = HIGH;

// Valores procesados (centrados: -512 a +512)
static int joy_centered_x = 0;
static int joy_centered_y = 0;

// Valores en porcentaje (-100 a +100)
static int joy_percent_x = 0;
static int joy_percent_y = 0;

// Estado del botón
static bool joy_button_pressed = false;
static bool joy_button_previous = HIGH;
static unsigned long joy_last_button_change = 0;

// Modo tanque
static bool joy_tank_mode = false;

// Control de tiempo
static unsigned long joy_last_update = 0;

// Estado de inicialización
static bool joy_initialized = false;

// Variables para monitoreo continuo
static bool joy_continuous_monitor = false;
static unsigned long joy_last_monitor = 0;
static const unsigned long joy_monitor_interval = 100; // 100ms entre reportes

// ============================================================================
// ==================== FUNCIONES INTERNAS DEL MÓDULO =========================
// ============================================================================

/**
 * FUNCIÓN: joy_readRawValues
 * Lee los valores RAW del hardware del joystick
 */
void joy_readRawValues() {
  joy_raw_x = analogRead(JOYSTICK_X_PIN);
  joy_raw_y = analogRead(JOYSTICK_Y_PIN);
  joy_raw_sw = digitalRead(JOYSTICK_SW_PIN);
}

/**
 * FUNCIÓN: joy_processValues
 * Procesa valores RAW a centrados y porcentajes
 */
void joy_processValues() {
  // Centrar valores (0-1023 -> -512 a +512)
  joy_centered_x = joy_raw_x - JOY_CENTER_VALUE;
  joy_centered_y = joy_raw_y - JOY_CENTER_VALUE;
  
  // Aplicar zona muerta
  if (abs(joy_centered_x) < JOY_DEADZONE) {
    joy_centered_x = 0;
  }
  if (abs(joy_centered_y) < JOY_DEADZONE) {
    joy_centered_y = 0;
  }
  
  // Convertir a porcentaje (-100 a +100)
  joy_percent_x = map(joy_centered_x, -512, 512, -100, 100);
  joy_percent_y = map(joy_centered_y, -512, 512, -100, 100);
  
  // Limitar a rango válido
  joy_percent_x = constrain(joy_percent_x, -100, 100);
  joy_percent_y = constrain(joy_percent_y, -100, 100);
}

/**
 * FUNCIÓN: joy_detectButtonPress
 * Detecta presión del botón con anti-rebote
 * RETORNA: true si se detectó presión nueva
 */
bool joy_detectButtonPress() {
  bool button_changed = false;
  
  // Detectar flanco descendente (botón presionado)
  if (joy_button_previous == HIGH && joy_raw_sw == LOW) {
    // Verificar tiempo de anti-rebote
    if (millis() - joy_last_button_change > JOY_DEBOUNCE_DELAY) {
      joy_button_pressed = true;
      button_changed = true;
      joy_last_button_change = millis();
      
      #ifdef ENABLE_JOYSTICK_DEBUG
        DEBUG_PRINTLN("=== BOTÓN JOYSTICK PRESIONADO ===");
      #endif
    }
  }
  // Detectar flanco ascendente (botón liberado)
  else if (joy_button_previous == LOW && joy_raw_sw == HIGH) {
    joy_button_pressed = false;
    joy_last_button_change = millis();
  }
  
  joy_button_previous = joy_raw_sw;
  return button_changed;
}

/**
 * FUNCIÓN: joy_toggleTankMode
 * Alterna el modo tanque
 */
void joy_toggleTankMode() {
  joy_tank_mode = !joy_tank_mode;
  
  #ifdef ENABLE_JOYSTICK_DEBUG
    if (joy_tank_mode) {
      DEBUG_PRINTLN(">>> MODO TANQUE ACTIVADO <<<");
    } else {
      DEBUG_PRINTLN(">>> MODO TANQUE DESACTIVADO <<<");
    }
  #endif
}

/**
 * FUNCIÓN: joy_interpretDirection
 * Interpreta la dirección del joystick
 * RETORNA: String con la dirección
 */
String joy_interpretDirection() {
  String direccion = "";
  
  if (abs(joy_centered_x) < JOY_DEADZONE && abs(joy_centered_y) < JOY_DEADZONE) {
    direccion = "CENTRO";
  } else {
    // Eje X: Adelante/Atrás
    if (joy_centered_x > JOY_DEADZONE) {
      direccion += "ADELANTE ";
    }
    if (joy_centered_x < -JOY_DEADZONE) {
      direccion += "ATRAS ";
    }
    
    // Eje Y: Izquierda/Derecha (giro)
    if (joy_centered_y > JOY_DEADZONE) {
      direccion += "DERECHA";
    }
    if (joy_centered_y < -JOY_DEADZONE) {
      direccion += "IZQUIERDA";
    }
  }
  
  return direccion;
}

// ============================================================================
// ==================== FUNCIONES PÚBLICAS DEL MÓDULO =========================
// ============================================================================

/**
 * FUNCIÓN: joy_initialize
 * Inicializa el hardware del joystick
 */
void joy_initialize() {
  #ifdef ENABLE_JOYSTICK_DEBUG
    DEBUG_PRINTLN("=================================");
    DEBUG_PRINTLN("  INICIALIZANDO JOYSTICK MODULE  ");
    DEBUG_PRINTLN("=================================");
  #endif
  
  // Configurar pines
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  
  // Leer valores iniciales
  joy_readRawValues();
  joy_processValues();
  
  #ifdef ENABLE_JOYSTICK_DEBUG
    DEBUG_PRINTLN(">>> Test inicial de hardware <<<");
    for(int i = 0; i < 3; i++) {
      joy_readRawValues();
      DEBUG_PRINT("Test "); DEBUG_PRINT(i+1); DEBUG_PRINT(": ");
      DEBUG_PRINT("X="); DEBUG_PRINT(joy_raw_x);
      DEBUG_PRINT(", Y="); DEBUG_PRINT(joy_raw_y);
      DEBUG_PRINT(", SW="); DEBUG_PRINTLN(joy_raw_sw);
      delay(100);
    }
    DEBUG_PRINTLN(">>> Test completado <<<");
  #endif
  
  joy_initialized = true;
  
  #ifdef ENABLE_JOYSTICK_DEBUG
    DEBUG_PRINTLN("✓ Joystick inicializado correctamente");
    DEBUG_PRINT("  - Pin SW: "); DEBUG_PRINTLN(JOYSTICK_SW_PIN);
    DEBUG_PRINT("  - Pin Y:  "); DEBUG_PRINTLN(JOYSTICK_Y_PIN);
    DEBUG_PRINT("  - Pin X:  "); DEBUG_PRINTLN(JOYSTICK_X_PIN);
    DEBUG_PRINT("  - Zona muerta: ±"); DEBUG_PRINTLN(JOY_DEADZONE);
    DEBUG_PRINTLN("=================================");
  #endif
}

/**
 * FUNCIÓN: updateJoystick
 * Actualiza el estado del joystick (llamar en loop)
 */
void updateJoystick() {
  if (!joy_initialized) return;
  
  // Control de tiempo de actualización
  if (millis() - joy_last_update < JOY_UPDATE_INTERVAL) {
    return;
  }
  joy_last_update = millis();
  
  // Leer valores del hardware
  joy_readRawValues();
  
  // Procesar valores
  joy_processValues();
  
  // Detectar presión del botón
  if (joy_detectButtonPress()) {
    joy_toggleTankMode();
  }
  
  // Monitoreo continuo si está activo
  #ifdef ENABLE_JOYSTICK_DEBUG
    if (joy_continuous_monitor && (millis() - joy_last_monitor >= joy_monitor_interval)) {
      DEBUG_PRINT("JOY: X="); DEBUG_PRINT(joy_percent_x);
      DEBUG_PRINT("%, Y="); DEBUG_PRINT(joy_percent_y);
      DEBUG_PRINT("%, SW="); DEBUG_PRINT(joy_button_pressed ? "PRESS" : "FREE");
      DEBUG_PRINT(", TANK="); DEBUG_PRINTLN(joy_tank_mode ? "ON" : "OFF");
      joy_last_monitor = millis();
    }
  #endif
}

/**
 * FUNCIÓN: joy_printStatus
 * Muestra el estado completo del joystick
 */
void joy_printStatus() {
  if (!joy_initialized) {
    DEBUG_PRINTLN("Joystick no inicializado");
    return;
  }
  
  DEBUG_PRINTLN("┌─────────────────────────────────┐");
  DEBUG_PRINTLN("│    ESTADO DEL JOYSTICK          │");
  DEBUG_PRINTLN("├─────────────────────────────────┤");
  
  // Valores RAW
  DEBUG_PRINT("│ X RAW: "); DEBUG_PRINT(joy_raw_x);
  DEBUG_PRINT(" ("); DEBUG_PRINT(joy_percent_x);
  DEBUG_PRINTLN("%)              │");
  
  DEBUG_PRINT("│ Y RAW: "); DEBUG_PRINT(joy_raw_y);
  DEBUG_PRINT(" ("); DEBUG_PRINT(joy_percent_y);
  DEBUG_PRINTLN("%)              │");
  
  DEBUG_PRINT("│ Botón: ");
  DEBUG_PRINT(joy_button_pressed ? "PRESIONADO" : "LIBRE");
  DEBUG_PRINTLN("           │");
  
  DEBUG_PRINT("│ Modo Tanque: ");
  DEBUG_PRINT(joy_tank_mode ? "ACTIVADO" : "DESACTIVADO");
  DEBUG_PRINTLN("      │");
  
  // Dirección interpretada
  DEBUG_PRINTLN("├─────────────────────────────────┤");
  DEBUG_PRINT("│ Dirección: ");
  String dir = joy_interpretDirection();
  DEBUG_PRINT(dir);
  for(int i = dir.length(); i < 18; i++) DEBUG_PRINT(" ");
  DEBUG_PRINTLN("│");
  
  DEBUG_PRINTLN("└─────────────────────────────────┘");
}

/**
 * FUNCIÓN: joy_printRawData
 * Muestra datos RAW compactos
 */
void joy_printRawData() {
  DEBUG_PRINT("X="); DEBUG_PRINT(joy_raw_x);
  DEBUG_PRINT(", Y="); DEBUG_PRINT(joy_raw_y);
  DEBUG_PRINT(", SW="); DEBUG_PRINT(joy_raw_sw);
  DEBUG_PRINT(", TANK="); DEBUG_PRINTLN(joy_tank_mode ? "ON" : "OFF");
}

/**
 * FUNCIÓN: joy_runHardwareTest
 * Ejecuta test de hardware del joystick
 */
void joy_runHardwareTest() {
  DEBUG_PRINTLN(">>> TEST DE HARDWARE JOYSTICK <<<");
  DEBUG_PRINTLN("Leyendo 10 valores...");
  
  for(int i = 0; i < 10; i++) {
    int x_test = analogRead(JOYSTICK_X_PIN);
    int y_test = analogRead(JOYSTICK_Y_PIN);
    int sw_test = digitalRead(JOYSTICK_SW_PIN);
    
    DEBUG_PRINT("Test "); DEBUG_PRINT(i+1); DEBUG_PRINT(": ");
    DEBUG_PRINT("X="); DEBUG_PRINT(x_test);
    DEBUG_PRINT(", Y="); DEBUG_PRINT(y_test);
    DEBUG_PRINT(", SW="); DEBUG_PRINTLN(sw_test);
    delay(200);
  }
  
  DEBUG_PRINTLN(">>> TEST COMPLETADO <<<");
}

/**
 * FUNCIÓN: joy_startContinuousMonitor
 * Inicia monitoreo continuo
 */
void joy_startContinuousMonitor() {
  joy_continuous_monitor = true;
  DEBUG_PRINTLN(">>> Monitoreo continuo INICIADO <<<");
}

/**
 * FUNCIÓN: joy_stopContinuousMonitor
 * Detiene monitoreo continuo
 */
void joy_stopContinuousMonitor() {
  joy_continuous_monitor = false;
  DEBUG_PRINTLN(">>> Monitoreo continuo DETENIDO <<<");
}

/**
 * FUNCIÓN: joy_resetTankMode
 * Resetea el modo tanque a OFF
 */
void joy_resetTankMode() {
  joy_tank_mode = false;
  DEBUG_PRINTLN("Modo tanque reseteado a OFF");
}

// ============================================================================
// ===================== FUNCIONES GETTER (API PÚBLICA) =======================
// ============================================================================

/**
 * Obtener valores RAW del joystick
 */
int getJoystickRawX() { return joy_raw_x; }
int getJoystickRawY() { return joy_raw_y; }
int getJoystickRawSW() { return joy_raw_sw; }

/**
 * Obtener valores centrados (-512 a +512)
 */
int getJoystickCenteredX() { return joy_centered_x; }
int getJoystickCenteredY() { return joy_centered_y; }

/**
 * Obtener valores en porcentaje (-100 a +100)
 */
int getJoystickPercentX() { return joy_percent_x; }
int getJoystickPercentY() { return joy_percent_y; }

/**
 * Obtener estados booleanos
 */
bool isJoystickButtonPressed() { return joy_button_pressed; }
bool isJoystickTankMode() { return joy_tank_mode; }
bool isJoystickInitialized() { return joy_initialized; }

/**
 * Obtener dirección interpretada
 */
String getJoystickDirection() { return joy_interpretDirection(); }

/**
 * Control del modo tanque
 */
void setJoystickTankMode(bool enabled) { 
  joy_tank_mode = enabled;
  #ifdef ENABLE_JOYSTICK_DEBUG
    DEBUG_PRINT("Modo tanque: ");
    DEBUG_PRINTLN(enabled ? "ACTIVADO" : "DESACTIVADO");
  #endif
}

/**
 * FUNCIÓN: joy_calculateMotorSpeeds
 * Calcula velocidades de motores según modo
 * PARÁMETROS:
 *   - left_speed: puntero para velocidad motor izquierdo (-100 a +100)
 *   - right_speed: puntero para velocidad motor derecho (-100 a +100)
 * RETORNA: true si hay movimiento, false si está en zona muerta
 */
bool joy_calculateMotorSpeeds(int* left_speed, int* right_speed) {
  if (!joy_initialized) {
    *left_speed = 0;
    *right_speed = 0;
    return false;
  }
  
  // Verificar si está en zona muerta
  if (joy_percent_x == 0 && joy_percent_y == 0) {
    *left_speed = 0;
    *right_speed = 0;
    return false;
  }
  
  if (joy_tank_mode) {
    // MODO TANQUE: Control diferencial
    // X = velocidad base
    // Y = diferencia entre motores (giro)
    
    int base_speed = joy_percent_x;     // Adelante/Atrás
    int turn_delta = joy_percent_y;     // Giro Izq/Der
    
    // Calcular velocidades con mezcla
    *left_speed = base_speed - (turn_delta / 2);
    *right_speed = base_speed + (turn_delta / 2);
    
    // Limitar a rango válido
    *left_speed = constrain(*left_speed, -100, 100);
    *right_speed = constrain(*right_speed, -100, 100);
    
  } else {
    // MODO NORMAL: Ambos motores igual velocidad
    // Solo usa eje X (adelante/atrás)
    *left_speed = joy_percent_x;
    *right_speed = joy_percent_x;
  }
  
  return true;
}

#endif // JOYSTICK_MODULE_H
