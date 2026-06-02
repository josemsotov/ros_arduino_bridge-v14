/**
 * MOTOR-INTERFACE-V-13 ODOMETRY MODULE
 * Smart Golf Trolley - Fusión Hall + MPU9250 con Filtro Complementario
 *
 * ╔══════════════════════════════════════════════════════════════════════╗
 * ║  ARQUITECTURA DE FUSIÓN SENSORIAL                                   ║
 * ╠══════════════════════════════════════════════════════════════════════╣
 * ║                                                                      ║
 * ║  Hall Sensors  →  velocidades lineales con signo (m/s)              ║
 * ║  MPU9250 gyroZ →  velocidad angular (rad/s), sin deriva a c.p.      ║
 * ║                                                                      ║
 * ║  Filtro complementario:                                              ║
 * ║    yaw_rate = α·gyroZ + (1-α)·yaw_rate_odom                         ║
 * ║    yaw      = ∫ yaw_rate · dt                                        ║
 * ║                                                                      ║
 * ║  α≈0.98: gyro domina a corto plazo (preciso, sin ruido de rueda)    ║
 * ║           odometría corrige deriva del gyro a largo plazo            ║
 * ║                                                                      ║
 * ║  Posición x,y integrada para publicación de odometría a ROS2        ║
 * ╚══════════════════════════════════════════════════════════════════════╝
 *
 * DEPENDENCIAS:
 *   - Hall_Sensors.h  (leftHallSigned, rightHallSigned)
 *   - MPU9250.h       (mpu_getGyroZ(), mpu_isReady())  [opcional]
 *   - Pins.h          (WHEEL_BASE_DISTANCE_M, etc.)
 *
 * COMANDOS SERIAL:
 *   ODOM          → Estado y pose actual
 *   ODOM_RESET    → Resetear pose a (0,0,0°)
 *   ODOM_DEBUG    → Toggle salida continua
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#if defined(ENABLE_HALL_SENSORS)

// Forward declarations — variables definidas en Core_Functions.h
// (Odometry.h se incluye ANTES de Core_Functions.h en Modules.h)
extern volatile int32_t leftHallSigned;
extern volatile int32_t rightHallSigned;

//===========================================================================
//========================== CONFIGURACIÓN ==================================
//===========================================================================

// Geometría del robot (coincide con Robot_States.h)
#define ODOM_WHEEL_BASE_M      0.82f    // Distancia entre centros de rueda (m)
#define ODOM_WHEEL_DIAMETER_M  0.20f    // Diámetro de rueda (m)
#define ODOM_WHEEL_RADIUS_M    0.10f    // Radio de rueda (m)
#define ODOM_PPR               45.0f    // Pulsos por revolución Hall
#define ODOM_DIST_PER_PULSE    (3.14159265f * ODOM_WHEEL_DIAMETER_M / ODOM_PPR)
                                        // = 0.01396 m/pulso

// Filtro complementario
// α = fracción que aporta el gyro. (1-α) aporta la odometría.
// Sube α → más peso al gyro (respuesta rápida, puede derivar)
// Baja α → más peso a odometría (estable a largo plazo, sensible a patinaje)
#define ODOM_CF_ALPHA          0.98f

// Intervalo de actualización de odometría
#define ODOM_UPDATE_MS         20       // 50 Hz

// Debug
#define ODOM_DEBUG_MS          200      // 5 Hz cuando debug activo

//===========================================================================
//========================== VARIABLES GLOBALES =============================
//===========================================================================

// Pose del robot en el sistema de referencia del mundo
float odom_x     = 0.0f;   // metros
float odom_y     = 0.0f;   // metros
float odom_theta = 0.0f;   // radianes (yaw)

// Velocidades estimadas (publicadas a ROS2)
float odom_v     = 0.0f;   // velocidad lineal   m/s
float odom_w     = 0.0f;   // velocidad angular  rad/s

// Velocidades individuales de cada rueda (m/s, con signo)
float odom_v_left  = 0.0f;
float odom_v_right = 0.0f;

// Snapshots anteriores de los contadores firmados
int32_t odom_prevLeftSigned  = 0;
int32_t odom_prevRightSigned = 0;

// Tiempo
unsigned long odom_lastUpdateMs = 0;
unsigned long odom_lastDebugMs  = 0;

// Estado
bool odom_debugActive = false;
bool odom_initialized = false;

//===========================================================================
//========================== INICIALIZACIÓN =================================
//===========================================================================

void odom_initialize() {
  odom_x = 0.0f;
  odom_y = 0.0f;
  odom_theta = 0.0f;
  odom_v = 0.0f;
  odom_w = 0.0f;
  odom_v_left  = 0.0f;
  odom_v_right = 0.0f;

  // Capturar baseline de contadores firmados
  noInterrupts();
  odom_prevLeftSigned  = leftHallSigned;
  odom_prevRightSigned = rightHallSigned;
  interrupts();

  odom_lastUpdateMs = millis();
  odom_initialized  = true;

  Serial.println("[ODOM] Odometría inicializada. Pose: (0,0,0)");
}

//===========================================================================
//========================== ACTUALIZACIÓN PRINCIPAL ========================
//===========================================================================

/**
 * Llamar desde loop() — calcula velocidades y actualiza pose.
 * Usa Hall para velocidades lineales y MPU9250 (si disponible) para yaw.
 */
void odom_update() {
  if (!odom_initialized) return;

  unsigned long now = millis();
  if (now - odom_lastUpdateMs < ODOM_UPDATE_MS) return;

  float dt = (now - odom_lastUpdateMs) / 1000.0f;
  odom_lastUpdateMs = now;

  // ── Leer deltas de contadores firmados (atómico) ──────────────────────
  noInterrupts();
  int32_t snapLeft  = leftHallSigned;
  int32_t snapRight = rightHallSigned;
  interrupts();

  int32_t dLeft  = snapLeft  - odom_prevLeftSigned;
  int32_t dRight = snapRight - odom_prevRightSigned;
  odom_prevLeftSigned  = snapLeft;
  odom_prevRightSigned = snapRight;

  // ── Distancias recorridas por cada rueda (m) ──────────────────────────
  float distLeft  = dLeft  * ODOM_DIST_PER_PULSE;
  float distRight = dRight * ODOM_DIST_PER_PULSE;

  // ── Velocidades lineales de cada rueda (m/s) ──────────────────────────
  if (dt > 0.0f) {
    odom_v_left  = distLeft  / dt;
    odom_v_right = distRight / dt;
  }

  // ── Cinemática diferencial → v y w de la odometría ───────────────────
  float v_odom = (distRight + distLeft) / 2.0f / dt;         // m/s lineal
  float w_odom = (distRight - distLeft) / ODOM_WHEEL_BASE_M / dt; // rad/s angular

  // ── Fusión complementaria: gyro MPU9250 + odometría Hall ────────────
  // gyro: preciso a corto plazo, puede derivar
  // odom: estable a largo plazo, señal limpia con Hall alimentados
  float w_fused = w_odom;  // fallback si MPU no disponible

#ifdef ENABLE_MPU9250
  if (mpu_isReady()) {
    float w_gyro = mpu_getGyroZ() * (3.14159265f / 180.0f);  // °/s → rad/s
    w_fused = ODOM_CF_ALPHA * w_gyro + (1.0f - ODOM_CF_ALPHA) * w_odom;
  }
#endif

  odom_v = v_odom;
  odom_w = w_fused;

  // ── Integración de pose (modelo unicycle) ───────────────────────────
  float theta_mid = odom_theta + w_fused * dt * 0.5f;

  odom_x     += odom_v * cosf(theta_mid) * dt;
  odom_y     += odom_v * sinf(theta_mid) * dt;
  odom_theta += w_fused * dt;

  // Normalizar theta a [-π, π]
  while (odom_theta >  3.14159265f) odom_theta -= 2.0f * 3.14159265f;
  while (odom_theta < -3.14159265f) odom_theta += 2.0f * 3.14159265f;

  // ── Debug continuo ────────────────────────────────────────────────────
  if (odom_debugActive && (now - odom_lastDebugMs >= ODOM_DEBUG_MS)) {
    odom_lastDebugMs = now;
    Serial.print("[ODOM] x=");   Serial.print(odom_x, 3);
    Serial.print(" y=");         Serial.print(odom_y, 3);
    Serial.print(" th=");        Serial.print(odom_theta * 180.0f / 3.14159265f, 1);
    Serial.print("° v=");        Serial.print(odom_v, 3);
    Serial.print("m/s w=");      Serial.print(odom_w, 3);
    Serial.println("rad/s");
  }
}

//===========================================================================
//========================== GETTERS ========================================
//===========================================================================

float odom_getX()       { return odom_x; }
float odom_getY()       { return odom_y; }
float odom_getTheta()   { return odom_theta; }          // rad
float odom_getThetaDeg(){ return odom_theta * 180.0f / 3.14159265f; }  // grados
float odom_getV()       { return odom_v; }              // m/s
float odom_getW()       { return odom_w; }              // rad/s
float odom_getVLeft()   { return odom_v_left; }
float odom_getVRight()  { return odom_v_right; }

//===========================================================================
//========================== RESET ==========================================
//===========================================================================

void odom_reset() {
  odom_x = 0.0f;
  odom_y = 0.0f;
  odom_theta = 0.0f;
  odom_v = 0.0f;
  odom_w = 0.0f;

  noInterrupts();
  odom_prevLeftSigned  = leftHallSigned;
  odom_prevRightSigned = rightHallSigned;
  interrupts();

  Serial.println("[ODOM] Pose reseteada a (0, 0, 0°)");
}

//===========================================================================
//========================== PRINT STATUS ===================================
//===========================================================================

void odom_printStatus() {
  Serial.println("============ ODOMETRÍA STATUS ============");
  Serial.print("  Inicializada : "); Serial.println(odom_initialized ? "SI" : "NO ← problema!");
  Serial.print("  Pose   : x="); Serial.print(odom_x, 4);
  Serial.print(" m  y=");        Serial.print(odom_y, 4);
  Serial.print(" m  th=");       Serial.print(odom_getThetaDeg(), 2);
  Serial.println(" deg");
  Serial.print("  Vel    : v="); Serial.print(odom_v, 4);
  Serial.print(" m/s  w=");      Serial.print(odom_w, 4);
  Serial.println(" rad/s");
  Serial.print("  Ruedas : izq="); Serial.print(odom_v_left, 4);
  Serial.print(" m/s  der=");      Serial.print(odom_v_right, 4);
  Serial.println(" m/s");
  // Contadores crudos para diagnóstico
  noInterrupts();
  int32_t sL = leftHallSigned;
  int32_t sR = rightHallSigned;
  int32_t pL = odom_prevLeftSigned;
  int32_t pR = odom_prevRightSigned;
  interrupts();
  Serial.print("  Hall±  : izq="); Serial.print(sL);
  Serial.print("  der=");          Serial.println(sR);
  Serial.print("  Prev±  : izq="); Serial.print(pL);
  Serial.print("  der=");          Serial.println(pR);
  Serial.print("  Delta  : izq="); Serial.print(sL - pL);
  Serial.print("  der=");          Serial.println(sR - pR);
  Serial.print("  m/pulso: "); Serial.println(ODOM_DIST_PER_PULSE, 6);
#ifdef ENABLE_MPU9250
  Serial.print("  Fusion : CF alpha="); Serial.print(ODOM_CF_ALPHA);
  Serial.println(" (0.98*gyro + 0.02*odom)");
#else
  Serial.println("  Fusion : SOLO HALL (MPU deshabilitado)");
#endif
  Serial.println("==========================================");
}

#endif  // ENABLE_HALL_SENSORS
#endif  // ODOMETRY_H
