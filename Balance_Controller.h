/**
 * ============================================================================
 * BALANCE_CONTROLLER — MODO HOVERBOARD
 * Smart Golf Trolley — Robot diferencial con control por inclinación
 * ============================================================================
 *
 * FUNCIONAMIENTO:
 *   El MPU9250 mide el ángulo de inclinación (pitch) del chasis:
 *     • Inclinar hacia ADELANTE  → robot avanza
 *     • Inclinar hacia ATRÁS     → robot frena / retrocede
 *     • Girar el chasis (yaw)    → robot gira
 *
 *   El controlador inyecta los setpoints en ros2_linear_vel y ros2_angular_vel,
 *   que ya tienen el PID por rueda con feedforward implementado en ROS2_Bridge.h.
 *   No hay cambios en la capa de control de motores.
 *
 * ORIENTACIÓN ESPERADA DEL MPU (montado plano sobre el chasis):
 *   Eje X apunta hacia ADELANTE del robot
 *   Eje Y apunta hacia la IZQUIERDA
 *   Eje Z apunta hacia ARRIBA
 *
 *   pitch_accel = atan2(accelX, accelZ)
 *     → positivo cuando el robot se inclina hacia adelante
 *
 * AJUSTE SI EL ROBOT VA AL REVÉS:
 *   Cambiar HOVERBOARD_PITCH_SIGN a -1.0f en Configuration.h
 *
 * PARÁMETROS AJUSTABLES (en Configuration.h):
 *   HOVERBOARD_DEAD_ZONE_DEG   — zona muerta ±X° sin respuesta (default 2°)
 *   HOVERBOARD_MAX_TILT_DEG    — inclinación → velocidad máxima (default 15°)
 *   HOVERBOARD_MAX_VEL_MS      — velocidad lineal máxima m/s (default 0.8)
 *   HOVERBOARD_YAW_SCALE       — escala gyroZ (°/s) → rad/s (default 0.018)
 *   HOVERBOARD_FALL_ANGLE_DEG  — ángulo de caída → emergency stop (default 40°)
 *   HOVERBOARD_COMP_ALPHA      — alpha filtro complementario (default 0.98)
 *   HOVERBOARD_PITCH_SIGN      — invertir dirección (+1 o -1)
 *
 * COMANDOS SERIAL:
 *   hb on   — activar modo hoverboard
 *   hb off  — desactivar modo hoverboard
 *   hb cal  — recalibrar MPU (robot quieto y nivelado)
 *   hb stat — estado y ángulos actuales
 * ============================================================================
 */

#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)

#include <Arduino.h>

//===========================================================================
//==================== VALORES DEFAULT (si no están en Configuration.h) ====
//===========================================================================

#ifndef HOVERBOARD_DEAD_ZONE_DEG
  #define HOVERBOARD_DEAD_ZONE_DEG    2.0f
#endif
#ifndef HOVERBOARD_MAX_TILT_DEG
  #define HOVERBOARD_MAX_TILT_DEG    15.0f
#endif
#ifndef HOVERBOARD_MAX_VEL_MS
  #define HOVERBOARD_MAX_VEL_MS       0.8f
#endif
#ifndef HOVERBOARD_YAW_SCALE
  #define HOVERBOARD_YAW_SCALE        0.018f
#endif
#ifndef HOVERBOARD_FALL_ANGLE_DEG
  #define HOVERBOARD_FALL_ANGLE_DEG  40.0f
#endif
#ifndef HOVERBOARD_COMP_ALPHA
  #define HOVERBOARD_COMP_ALPHA       0.98f
#endif
#ifndef HOVERBOARD_PITCH_SIGN
  #define HOVERBOARD_PITCH_SIGN       1.0f
#endif

// Frecuencia de actualización del controlador (ms)
#define HOVERBOARD_UPDATE_MS  20    // 50 Hz

//===========================================================================
//==================== VARIABLES GLOBALES ===================================
//===========================================================================

bool  hoverboard_active    = false;   // true = modo hoverboard activo
bool  hoverboard_fallen    = false;   // true = se detectó caída, requiere reset manual
float hoverboard_linear    = 0.0f;   // setpoint calculado m/s
float hoverboard_angular   = 0.0f;   // setpoint calculado rad/s

static unsigned long hb_last_update = 0;

//===========================================================================
//==================== FUNCIONES DE CONTROL =================================
//===========================================================================

/**
 * Activa el modo hoverboard.
 * Recalibra el MPU para establecer la posición actual como referencia.
 */
void hoverboard_enable() {
  if (!mpu_isReady()) {
    Serial.println("[HB] ERROR: MPU no disponible. Verifica conexión I2C.");
    return;
  }
  mpu_calibrateOffsets();   // recalibra drift giroscopio con robot quieto/nivelado
  hoverboard_fallen  = false;
  hoverboard_active  = true;
  hoverboard_linear  = 0.0f;
  hoverboard_angular = 0.0f;
  Serial.println("[HB] Modo hoverboard ACTIVADO");
  Serial.print(  "[HB] Dead zone: ±"); Serial.print(HOVERBOARD_DEAD_ZONE_DEG, 1); Serial.println("°");
  Serial.print(  "[HB] Max tilt:  ±"); Serial.print(HOVERBOARD_MAX_TILT_DEG,  1); Serial.println("°");
  Serial.print(  "[HB] Max vel:    "); Serial.print(HOVERBOARD_MAX_VEL_MS,    1); Serial.println(" m/s");
}

/**
 * Desactiva el modo hoverboard y para los motores.
 */
void hoverboard_disable() {
  hoverboard_active  = false;
  hoverboard_linear  = 0.0f;
  hoverboard_angular = 0.0f;
  // Detener motores de forma segura
  setLeftMotor(0, true);
  setRightMotor(0, false);
  pid_reset_velocity();
  pid_per_wheel_reset();
  Serial.println("[HB] Modo hoverboard DESACTIVADO");
}

/**
 * Imprime el estado actual del controlador y los ángulos.
 */
void hoverboard_printStatus() {
  Serial.println("======= HOVERBOARD STATUS =======");
  Serial.print("[HB] Activo:   "); Serial.println(hoverboard_active  ? "SÍ" : "NO");
  Serial.print("[HB] Caído:    "); Serial.println(hoverboard_fallen  ? "SÍ — resetear con 'hb on'" : "NO");
  Serial.print("[HB] MPU listo:"); Serial.println(mpu_isReady()      ? "SÍ" : "NO");
  if (mpu_isReady()) {
    Serial.print("[HB] Pitch:    "); Serial.print(mpu_getPitch(),  2); Serial.println("°");
    Serial.print("[HB] Yaw:      "); Serial.print(mpu_getYaw(),    2); Serial.println("°");
    Serial.print("[HB] GyroZ:    "); Serial.print(mpu_getGyroZ(),  2); Serial.println("°/s");
    Serial.print("[HB] GyroY:    "); Serial.print(mpu_gyroY_f,     2); Serial.println("°/s");
    Serial.print("[HB] AccelX:   "); Serial.print(mpu_getAccelX(), 3); Serial.println(" g");
    Serial.print("[HB] AccelZ:   "); Serial.print(mpu_getAccelZ(), 3); Serial.println(" g");
  }
  Serial.print("[HB] Cmd lin:  "); Serial.print(hoverboard_linear,  3); Serial.println(" m/s");
  Serial.print("[HB] Cmd ang:  "); Serial.print(hoverboard_angular, 3); Serial.println(" rad/s");
  Serial.println("=================================");
}

/**
 * Actualiza el controlador hoverboard: lee pitch/gyroZ y calcula setpoints.
 * Llamar desde loop() — tiene su propio rate limit (HOVERBOARD_UPDATE_MS).
 * Cuando está activo, inyecta en ros2_linear_vel / ros2_angular_vel.
 */
void hoverboard_update() {
  if (!hoverboard_active) return;
  if (!mpu_isReady())     return;

  unsigned long now = millis();
  if (now - hb_last_update < HOVERBOARD_UPDATE_MS) return;
  hb_last_update = now;

  float pitch = mpu_getPitch() * HOVERBOARD_PITCH_SIGN;
  float gyroZ = mpu_getGyroZ();  // °/s

  // ── Detección de caída ────────────────────────────────────────────────────
  if (fabsf(pitch) > HOVERBOARD_FALL_ANGLE_DEG) {
    Serial.print("[HB] ⚠️ CAÍDA DETECTADA — pitch=");
    Serial.print(pitch, 1);
    Serial.println("°  Motores detenidos. Usa 'hb on' para reactivar.");
    hoverboard_fallen = true;
    hoverboard_disable();
    emergencyStop();
    return;
  }

  // ── Zona muerta ───────────────────────────────────────────────────────────
  float abs_pitch = fabsf(pitch);
  float linear    = 0.0f;

  if (abs_pitch > HOVERBOARD_DEAD_ZONE_DEG) {
    // Normalizar: 0.0 en el borde de la dead zone, 1.0 en MAX_TILT
    float range = HOVERBOARD_MAX_TILT_DEG - HOVERBOARD_DEAD_ZONE_DEG;
    float norm  = (abs_pitch - HOVERBOARD_DEAD_ZONE_DEG) / range;
    norm        = constrain(norm, 0.0f, 1.0f);
    linear      = (pitch > 0.0f ? 1.0f : -1.0f) * norm * HOVERBOARD_MAX_VEL_MS;
  }

  // ── Velocidad angular desde gyro Z ───────────────────────────────────────
  // Zona muerta pequeña para gyroZ (ruido en reposo ~0.5°/s)
  float angular = 0.0f;
  if (fabsf(gyroZ) > 1.5f) {
    angular = -gyroZ * HOVERBOARD_YAW_SCALE;   // negativo: girar CW → angular negativa
    angular = constrain(angular, -ROS2_MAX_ANGULAR_VEL, ROS2_MAX_ANGULAR_VEL);
  }

  hoverboard_linear  = linear;
  hoverboard_angular = angular;

  // ── Inyectar en el puente ROS2 ────────────────────────────────────────────
  // Se inyecta directamente en las variables del bridge, exactamente igual que
  // cuando llega un comando 'v' por serial. Así el PID por rueda ya existente
  // maneja el control real de los motores.
  ros2_linear_vel    = linear;
  ros2_angular_vel   = angular;
  ros2_last_cmd_time = now;    // evita el timeout de 1s del bridge
  ros2_connected     = true;

  // Habilitar robot si estaba inhabilitado
  if (currentRobotState != STATE_HABILITADO) {
    setStateHabilitado();
  }

  // Disparar el PID via ros2_processCmdVel directamente
  // Se construye un string 'v linear angular' y se llama como si fuera un
  // comando serial. Evita duplicar la lógica del PID.
  String cmd = "v ";
  cmd += String(linear,  4);
  cmd += " ";
  cmd += String(angular, 4);
  ros2_processCmdVel(cmd);
}

/**
 * Procesa comandos 'hb' recibidos por serial.
 * Llamar desde ros2_processCommand() o Serial_Command_Processor.
 */
void hoverboard_processCommand(String args) {
  args.trim();
  if (args == "on" || args == "ON") {
    hoverboard_enable();
  } else if (args == "off" || args == "OFF") {
    hoverboard_disable();
  } else if (args == "cal" || args == "CAL") {
    if (!mpu_isReady()) { Serial.println("[HB] MPU no disponible."); return; }
    Serial.println("[HB] Recalibrando MPU... (mantener robot quieto y nivelado)");
    mpu_calibrateOffsets();
    Serial.println("[HB] Calibración completada.");
  } else if (args == "stat" || args == "STATUS") {
    hoverboard_printStatus();
  } else {
    Serial.println("[HB] Comandos disponibles:");
    Serial.println("  hb on   — activar modo hoverboard");
    Serial.println("  hb off  — desactivar");
    Serial.println("  hb cal  — recalibrar MPU (quieto y nivelado)");
    Serial.println("  hb stat — estado y ángulos actuales");
  }
}

#endif  // ENABLE_HOVERBOARD_MODE && ENABLE_MPU9250
#endif  // BALANCE_CONTROLLER_H
