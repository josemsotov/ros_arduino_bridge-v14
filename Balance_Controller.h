/**
 * ============================================================================
 * BALANCE_CONTROLLER — ANTI-CAÍDA ACTIVA
 * Smart Golf Trolley — Equilibrio dinámico del robot diferencial
 * ============================================================================
 *
 * PROBLEMA QUE RESUELVE:
 *   Un robot diferencial pierde el equilibrio dinámico cuando:
 *     - Frena bruscamente   → inercia inclina el chasis hacia adelante
 *     - Sube/baja una rampa → gravedad inclina el chasis
 *     - Golpea un obstáculo → impulso inclina el chasis
 *
 *   Este módulo mide el ángulo de inclinación (pitch) del chasis con el
 *   MPU9250 y AÑADE una corrección de velocidad al comando del usuario
 *   que mueve las ruedas en la dirección de la inclinación, "siguiendo"
 *   el centro de gravedad para restablecer el equilibrio.
 *
 * LEY DE CONTROL (controlador PD sobre pitch):
 *   error      = pitch_actual - pitch_neutral    (° desde vertical calibrada)
 *   correction = Kp * error + Kd * gyroY         (m/s — derivada = gyroY directo)
 *   v_total    = v_usuario + correction           (se suma al comando actual)
 *
 * EJEMPLO:
 *   Robot avanzando a 0.4 m/s. Golpea una piedra → chasis se inclina +8°.
 *   correction = 0.025 * 8 + 0.002 * 40 = 0.28 m/s
 *   v_total = 0.4 + 0.28 = 0.68 m/s → ruedas aceleran → CG vuelve al eje.
 *
 * ZONAS DE ACTUACIÓN:
 *   |error| < HOVERBOARD_DEAD    → zona muerta, sin corrección
 *   DEAD < |error| < FALL_ANGLE  → corrección PD proporcional
 *   |error| > FALL_ANGLE         → caída detectada → emergency stop
 *
 * ACTIVACIÓN:
 *   Se activa AUTOMÁTICAMENTE al arrancar (tras calibración del MPU).
 *   Siempre en segundo plano mientras el robot está habilitado.
 *
 * COMANDOS SERIAL:
 *   hb on              — reactivar (tras caída o desactivación manual)
 *   hb off             — desactivar (solo para diagnóstico)
 *   hb cal             — recalibrar neutral (robot quieto y nivelado)
 *   hb cal front       — calibrar límite frontal en posición actual
 *   hb cal rear        — calibrar límite trasero en posición actual
 *   hb stat            — estado, ángulos y parámetros actuales
 *   hb kp <valor>      — ajustar Kp en runtime
 *   hb kd <valor>      — ajustar Kd en runtime
 *   hb front <grados>  — fijar límite frontal manualmente
 *   hb rear <grados>   — fijar límite trasero manualmente
 * ============================================================================
 */

#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)

#include <Arduino.h>

//===========================================================================
//==================== PARÁMETROS POR DEFECTO ==============================
//===========================================================================

// Ganancia proporcional: corrección en m/s por grado de inclinación
// 0.025 → a 10° de inclinación añade 0.25 m/s de corrección
#ifndef HOVERBOARD_KP
  #define HOVERBOARD_KP           0.025f
#endif

// Ganancia derivativa: corrección en m/s por (°/s) de velocidad angular
// Usa gyroY directamente — amortigua oscilaciones sin calcular derivadas
// 0.002 → a 50°/s añade 0.10 m/s de amortiguación
#ifndef HOVERBOARD_KD
  #define HOVERBOARD_KD           0.002f
#endif

// Zona muerta: inclinaciones menores a este ángulo no generan corrección
#ifndef HOVERBOARD_DEAD
  #define HOVERBOARD_DEAD         1.5f    // grados
#endif

// Ángulo máximo de corrección (corrección satura aquí, no para el robot)
#define BAL_MAX_CORRECTION        0.40f   // m/s

// Límite de caída frontal y trasero — valor por defecto del define en Configuration.h
// Se sobreescriben en runtime con 'hb cal front/rear' o 'hb front/rear <val>'
#ifndef HOVERBOARD_FRONT_LIMIT_DEG
  #define HOVERBOARD_FRONT_LIMIT_DEG  35.0f
#endif
#ifndef HOVERBOARD_REAR_LIMIT_DEG
  #define HOVERBOARD_REAR_LIMIT_DEG   35.0f
#endif
// Alias de retrocompatibilidad
#ifndef HOVERBOARD_FALL_ANGLE_DEG
  #define HOVERBOARD_FALL_ANGLE_DEG   HOVERBOARD_FRONT_LIMIT_DEG
#endif

// Intervalo de actualización del controlador
#define BAL_UPDATE_MS  20   // 50 Hz

//===========================================================================
//==================== VARIABLES GLOBALES ==================================
//===========================================================================

// Velocidad base del usuario — se actualiza con cada comando 'v'
// extern: declarado también en ROS2_Bridge.h para que el bridge lo actualice
float bal_base_linear  = 0.0f;
float bal_base_angular = 0.0f;

// Flag: cuando balance llama ros2_processCmdVel(), suprime la telemetría T
// para no inundar el serial a 50 Hz
bool  bal_suppress_telemetry = false;

// Estado del controlador
bool  balance_active      = false;
bool  balance_fallen      = false;   // true tras caída — requiere 'hb on'
float bal_setpoint        = 0.0f;   // pitch neutral calibrado (°)
float bal_Kp              = HOVERBOARD_KP;
float bal_Kd              = HOVERBOARD_KD;
float bal_last_correction = 0.0f;   // corrección aplicada en el último ciclo

// Límites de inclinación asimétricos (° relativos al setpoint)
// Positivo = frontal (robot cae hacia adelante)
// Negativo implícito = trasero (robot cae hacia atrás)
float bal_front_limit = HOVERBOARD_FRONT_LIMIT_DEG;  // calibrado con 'hb cal front'
float bal_rear_limit  = HOVERBOARD_REAR_LIMIT_DEG;   // calibrado con 'hb cal rear'
bool  bal_front_calibrated = false;  // true = límite frontal fue calibrado (no es default)
bool  bal_rear_calibrated  = false;  // true = límite trasero fue calibrado (no es default)

static unsigned long bal_last_update    = 0;
static unsigned long bal_last_telemetry = 0;

//===========================================================================
//==================== ACTIVACIÓN / DESACTIVACIÓN ==========================
//===========================================================================

void hoverboard_enable() {
  if (!mpu_isReady()) {
    Serial.println(F("[BAL] ERROR: MPU no disponible. Verifica I2C (SDA=20, SCL=21)."));
    return;
  }
  // Calibrar gyro drift + establecer el ángulo actual como neutral
  mpu_calibrateOffsets();
  bal_setpoint        = mpu_getPitch();
  balance_fallen      = false;
  balance_active      = true;
  bal_last_correction = 0.0f;
  Serial.println(F("[BAL] Anti-caída ACTIVADO"));
  Serial.print(  F("[BAL] Setpoint: ")); Serial.print(bal_setpoint, 2); Serial.println('°');
  Serial.print(  F("[BAL] Kp="));        Serial.print(bal_Kp, 4);
  Serial.print(  F("  Kd="));            Serial.print(bal_Kd, 4);
  Serial.print(  F("  Dead=±"));         Serial.print(HOVERBOARD_DEAD, 1); Serial.println('°');
  Serial.print(  F("[BAL] Lim FRONT: +")); Serial.print(bal_front_limit, 1);
  Serial.print(  bal_front_calibrated ? F("° (CAL)") : F("° (default)"));
  Serial.print(  F("   REAR: -"));  Serial.print(bal_rear_limit, 1);
  Serial.println(bal_rear_calibrated  ? F("° (CAL)") : F("° (default)"));
}

void hoverboard_disable() {
  balance_active      = false;
  bal_last_correction = 0.0f;
  Serial.println(F("[BAL] Anti-caída DESACTIVADO"));
}

//===========================================================================
//==================== DIAGNÓSTICO =========================================
//===========================================================================

void hoverboard_printStatus() {
  Serial.println("====== BALANCE ANTI-CAÍDA ======");
  Serial.print("[BAL] Activo:      "); Serial.println(balance_active ? "SÍ" : "NO");
  Serial.print("[BAL] Caído:       "); Serial.println(balance_fallen ? "SÍ — 'hb on' para reactivar" : "NO");
  Serial.print("[BAL] MPU:         "); Serial.println(mpu_isReady()  ? "OK" : "NO DETECTADO");
  if (mpu_isReady()) {
    float err = mpu_getPitch() - bal_setpoint;
    Serial.print("[BAL] Pitch abs:   "); Serial.print(mpu_getPitch(), 2); Serial.println("°");
    Serial.print("[BAL] Error pitch: "); Serial.print(err,            2); Serial.println("°");
    Serial.print("[BAL] GyroY:       "); Serial.print(mpu_gyroY_f,   2); Serial.println("°/s");
    Serial.print("[BAL] Corrección:  "); Serial.print(bal_last_correction, 3); Serial.println(" m/s");
  }
  Serial.print("[BAL] Cmd base:    v="); Serial.print(bal_base_linear,  3);
  Serial.print(" w=");                   Serial.println(bal_base_angular, 3);
  Serial.print("[BAL] Setpoint:    "); Serial.print(bal_setpoint, 2); Serial.println("°");
  Serial.print("[BAL] Kp="); Serial.print(bal_Kp, 4);
  Serial.print("  Kd=");     Serial.println(bal_Kd, 4);
  Serial.println("================================");
}

//===========================================================================
//==================== BUCLE DE CONTROL (50 Hz) ============================
//===========================================================================

void hoverboard_update() {
  if (!balance_active) return;
  if (!mpu_isReady())  return;
  // Solo actuar si el robot ya está habilitado por el usuario.
  // ros2_processCmdVel() auto-habilita los motores; sin este guard
  // el balance encendería los motores al boot solo con >1.5° de tilt.
  if (currentRobotState != STATE_HABILITADO) return;

  unsigned long now = millis();
  if (now - bal_last_update < BAL_UPDATE_MS) return;
  bal_last_update = now;

  // Error de inclinación respecto al neutral calibrado
  float pitch_error = mpu_getPitch() - bal_setpoint;
  float gyroY       = mpu_gyroY_f;   // °/s — derivada directa del pitch

  // ── Detección de caída ASIMÉTRICA ─────────────────────────────────────────
  // Límite frontal (+) y trasero (-) independientes, calibrables en runtime
  if (pitch_error > bal_front_limit) {
    Serial.print(F("[BAL] CAIDA FRONTAL pitch_err="));
    Serial.print(pitch_error, 1);
    Serial.print(F(" > lim=")); Serial.print(bal_front_limit, 1);
    Serial.println(F(" — 'hb on' para reactivar."));
    balance_fallen = true;
    hoverboard_disable();
    emergencyStop();
    return;
  }
  if (pitch_error < -bal_rear_limit) {
    Serial.print(F("[BAL] CAIDA TRASERA pitch_err="));
    Serial.print(pitch_error, 1);
    Serial.print(F(" < lim=-")); Serial.print(bal_rear_limit, 1);
    Serial.println(F(" — 'hb on' para reactivar."));
    balance_fallen = true;
    hoverboard_disable();
    emergencyStop();
    return;
  }

  // ── Zona muerta ────────────────────────────────────────────────────────────
  float correction = 0.0f;
  if (fabsf(pitch_error) > HOVERBOARD_DEAD) {
    // PD: proporcional al ángulo + derivativa (gyroY amortigua oscilaciones)
    correction = bal_Kp * pitch_error + bal_Kd * gyroY;
    correction = constrain(correction, -BAL_MAX_CORRECTION, BAL_MAX_CORRECTION);
  }
  bal_last_correction = correction;

  // ── Velocidad final = comando del usuario + corrección anti-caída ─────────
  // bal_base_linear se actualiza en ROS2_Bridge.h cada vez que llega 'v linear w'
  float final_linear  = constrain(bal_base_linear  + correction,
                                  -ROS2_MAX_LINEAR_VEL, ROS2_MAX_LINEAR_VEL);
  float final_angular = bal_base_angular;

  // ── Enviar al PID por rueda vía ros2_processCmdVel ────────────────────────
  // La telemetría T interna se suprime (bal_suppress_telemetry=true) para
  // no inundar el serial a 50 Hz. El balance emite su propia línea B a 10 Hz.
  bal_suppress_telemetry = true;
  String cmd = "v ";
  cmd += String(final_linear,  4);
  cmd += " ";
  cmd += String(final_angular, 4);
  ros2_processCmdVel(cmd);
  ros2_last_cmd_time = now;   // evita el timeout de 1s del bridge
  ros2_connected     = true;
  bal_suppress_telemetry = false;

  // ── Telemetría de balance (10 Hz) ─────────────────────────────────────────
  // Formato: B err=<pitch_error> cor=<correction> base=<user_cmd> fin=<final>
  if (now - bal_last_telemetry >= 100) {
    bal_last_telemetry = now;
    Serial.print("B err=");  Serial.print(pitch_error,      2);
    Serial.print(" gy=");    Serial.print(gyroY,             1);
    Serial.print(" cor=");   Serial.print(correction,        3);
    Serial.print(" base=");  Serial.print(bal_base_linear,   3);
    Serial.print(" fin=");   Serial.print(final_linear,      3);
    Serial.print(" fl=");    Serial.print(bal_front_limit,   1);
    Serial.print(" rl=");    Serial.print(bal_rear_limit,    1);
    Serial.print(" Lrpm=");  Serial.print((int)currentSpeedLeftHall);
    Serial.print(" Rrpm=");  Serial.print((int)currentSpeedRightHall);
    #ifdef ENABLE_CURRENT_SENSORS
    Serial.print(F(" LmA="));  Serial.print(current_left_A,  2);
    Serial.print(F(" RmA="));  Serial.print(current_right_A, 2);
    #endif
    Serial.println();
  }
}

//===========================================================================
//==================== PROCESAMIENTO DE COMANDOS ===========================
//===========================================================================

void hoverboard_processCommand(String args) {
  args.trim();
  if (args.startsWith("on")) {
    hoverboard_enable();
  } else if (args.startsWith("off")) {
    hoverboard_disable();
  } else if (args.startsWith("cal front")) {
    // Calibrar límite frontal: inclinar el robot al ángulo máximo seguro hacia adelante
    if (!mpu_isReady()) { Serial.println(F("[BAL] MPU no disponible.")); return; }
    float current_err = mpu_getPitch() - bal_setpoint;
    if (current_err <= 0.0f) {
      Serial.println(F("[BAL] ERROR: inclina el robot HACIA ADELANTE antes de calibrar el limite frontal."));
      return;
    }
    bal_front_limit      = current_err;
    bal_front_calibrated = true;
    Serial.print(F("[BAL] Limite FRONTAL calibrado: +"));
    Serial.print(bal_front_limit, 2); Serial.println(F(" grd desde setpoint"));
  } else if (args.startsWith("cal rear")) {
    // Calibrar límite trasero: inclinar el robot al ángulo máximo seguro hacia atrás
    if (!mpu_isReady()) { Serial.println(F("[BAL] MPU no disponible.")); return; }
    float current_err = mpu_getPitch() - bal_setpoint;
    if (current_err >= 0.0f) {
      Serial.println(F("[BAL] ERROR: inclina el robot HACIA ATRAS antes de calibrar el limite trasero."));
      return;
    }
    bal_rear_limit      = -current_err;   // guardar como valor positivo
    bal_rear_calibrated = true;
    Serial.print(F("[BAL] Limite TRASERO calibrado: -"));
    Serial.print(bal_rear_limit, 2); Serial.println(F(" grd desde setpoint"));
  } else if (args.startsWith("cal")) {
    if (!mpu_isReady()) { Serial.println(F("[BAL] MPU no disponible.")); return; }
    mpu_calibrateOffsets();
    bal_setpoint = mpu_getPitch();
    Serial.print(F("[BAL] Recalibrado. Setpoint: "));
    Serial.print(bal_setpoint, 2); Serial.println(F(" grd"));
  } else if (args.startsWith("front ") || args.startsWith("front\t")) {
    float v = args.substring(6).toFloat();
    if (v <= 0.0f) { Serial.println(F("[BAL] front debe ser > 0")); return; }
    bal_front_limit      = v;
    bal_front_calibrated = false;
    Serial.print(F("[BAL] Limite FRONTAL: +")); Serial.print(bal_front_limit, 2); Serial.println(F(" grd"));
  } else if (args.startsWith("rear ") || args.startsWith("rear\t")) {
    float v = args.substring(5).toFloat();
    if (v <= 0.0f) { Serial.println(F("[BAL] rear debe ser > 0")); return; }
    bal_rear_limit      = v;
    bal_rear_calibrated = false;
    Serial.print(F("[BAL] Limite TRASERO: -")); Serial.print(bal_rear_limit, 2); Serial.println(F(" grd"));
  } else if (args.startsWith("stat")) {
    hoverboard_printStatus();
  } else if (args.startsWith("kp ") || args.startsWith("kp\t")) {
    bal_Kp = args.substring(3).toFloat();
    Serial.print(F("[BAL] Kp = ")); Serial.println(bal_Kp, 4);
  } else if (args.startsWith("kd ") || args.startsWith("kd\t")) {
    bal_Kd = args.substring(3).toFloat();
    Serial.print(F("[BAL] Kd = ")); Serial.println(bal_Kd, 4);
  } else {
    Serial.println(F("[BAL] Comandos:"));
    Serial.println(F("  hb on              - activar anti-caida"));
    Serial.println(F("  hb off             - desactivar"));
    Serial.println(F("  hb cal             - recalibrar neutral (quieto y nivelado)"));
    Serial.println(F("  hb cal front       - calibrar limite frontal (inclinado adelante)"));
    Serial.println(F("  hb cal rear        - calibrar limite trasero (inclinado atras)"));
    Serial.println(F("  hb front <grados>  - fijar limite frontal manualmente"));
    Serial.println(F("  hb rear  <grados>  - fijar limite trasero manualmente"));
    Serial.println(F("  hb stat            - angulos, limites y parametros"));
    Serial.println(F("  hb kp <valor>      - ajustar Kp (m/s por grado)"));
    Serial.println(F("  hb kd <valor>      - ajustar Kd (m/s por grado/s)"));
  }
}

#endif  // ENABLE_HOVERBOARD_MODE && ENABLE_MPU9250
#endif  // BALANCE_CONTROLLER_H
