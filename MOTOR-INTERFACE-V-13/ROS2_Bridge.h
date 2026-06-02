/**
 * ============================================================================
 * MOTOR-INTERFACE-V-13 - ROS2 BRIDGE MODULE
 * Smart Golf Trolley - Interfaz con ROS2 (Raspberry Pi 5) - VERSIÓN LIMPIA
 * ============================================================================
 * 
 * PROTOCOLO:
 * Arduino <- Raspberry Pi:
 *   v <linear_vel> <angular_vel>  : Comando de velocidad (m/s, rad/s)
 *   e                              : Request encoder counts (con signo)
 *   o                              : Request odometry pose + velocidades
 *   i                              : Request IMU data (MPU9250: gyroZ + accelX)
 *   r                              : Reset encoders y pose
 *   s                              : Request status
 *   c                              : Calibración PID
 *
 * Arduino -> Raspberry Pi:
 *   e <left_signed> <right_signed> : Encoder counts con signo
 *   o <x> <y> <theta> <v> <w>     : Pose (m,m,rad) + velocidades (m/s,rad/s)
 *   i <gz_rad_s> <ax_ms2>          : IMU: gyroZ (rad/s), accelX (m/s²) — Perfil A
 *   s <state>                      : Status (0=OK, 1=ERROR, 2=WARNING)
 * 
 * Autor: JMS 2025
 * ============================================================================
 */

#ifndef ROS2_BRIDGE_H
#define ROS2_BRIDGE_H

#ifdef ENABLE_ROS2_BRIDGE

#include <Arduino.h>

//===========================================================================
//==================== CONFIGURACIÓN ROS2 ===================================
//===========================================================================

#define ROS2_CMD_TIMEOUT      1000   // Timeout seguridad: para motores si no llega cmd_vel (ms)
#define ROS2_MAX_LINEAR_VEL   1.0    // m/s
#define ROS2_MAX_ANGULAR_VEL  2.0    // rad/s
#define ROS2_BUFFER_SIZE      128

//===========================================================================
//==================== VARIABLES GLOBALES ROS2 ==============================
//===========================================================================

bool ros2_connected = false;
unsigned long ros2_last_cmd_time = 0;

float ros2_linear_vel = 0.0;
float ros2_angular_vel = 0.0;

char ros2_buffer[ROS2_BUFFER_SIZE];
int ros2_buffer_index = 0;

// Puerto activo para respuestas al Pi5.
// Cambiar con USE_BT / USE_USB desde Serial_Command_Processor.
Stream* ros2_port     = &Serial;   // &Serial = USB (defecto)  |  &BT_SERIAL = Bluetooth
bool    ros2_using_bt = false;

enum ROS2_Status {
  ROS2_STATUS_OK = 0,
  ROS2_STATUS_ERROR = 1,
  ROS2_STATUS_WARNING = 2
};

ROS2_Status ros2_status = ROS2_STATUS_OK;

// ─── ANTI-STICIÓN: boost PWM si la rueda no arranca ──────────────────────
// Si una rueda tiene PWM > 0 pero no gira (Hall < umbral mínimo),
// se incrementa el PWM progresivamente hasta que venza la inercia estática.
// En cuanto la rueda gira, se restaura el PWM base solicitado.
static int           as_base_left   = 0;    // PWM base motor izquierdo
static int           as_base_right  = 0;    // PWM base motor derecho
static bool          as_dir_left    = true; // dirección izquierdo
static bool          as_dir_right   = true; // dirección derecho
static int           as_boost_left  = 0;    // boost acumulado izquierdo
static int           as_boost_right = 0;    // boost acumulado derecho
static unsigned long as_last_ms     = 0;    // timestamp última ejecución
static unsigned long as_cmd_ms      = 0;    // timestamp último cmd_vel
#define AS_MIN_SPEED      0.5f   // RPM mínimas para considerar que la rueda gira
#define AS_BOOST_STEP     3      // PWM que se añade por ciclo si hay stall
#define AS_MAX_BOOST      30     // boost máximo acumulado
#define AS_PERIOD_MS      80     // ms entre comprobaciones
#define AS_START_DELAY_MS 200    // ms de espera tras cmd_vel antes de activar

//===========================================================================
//==================== FUNCIONES DE INICIALIZACIÓN ==========================
//===========================================================================

void ros2_initialize() {
  ros2_connected     = false;
  ros2_linear_vel    = 0.0;
  ros2_angular_vel   = 0.0;
  ros2_buffer_index  = 0;
  ros2_last_cmd_time = millis();
  ros2_status        = ROS2_STATUS_OK;
  ros2_port          = &Serial;
  ros2_using_bt      = false;

  Serial.println(F("[ROS2] Bridge inicializado — puerto activo: USB"));
  Serial.println(F("[ROS2] Usa USE_BT / USE_USB para cambiar puerto."));
}

//===========================================================================
//==================== PROCESAMIENTO DE COMANDOS ============================
//===========================================================================

void ros2_processCmdVel(String cmd) {
  int space1 = cmd.indexOf(' ');
  int space2 = cmd.indexOf(' ', space1 + 1);
  
  if (space1 > 0 && space2 > 0) {
    String linear_str = cmd.substring(space1 + 1, space2);
    String angular_str = cmd.substring(space2 + 1);
    
    float linear = linear_str.toFloat();
    float angular = angular_str.toFloat();
    
    linear = constrain(linear, -ROS2_MAX_LINEAR_VEL, ROS2_MAX_LINEAR_VEL);
    angular = constrain(angular, -ROS2_MAX_ANGULAR_VEL, ROS2_MAX_ANGULAR_VEL);
    
    ros2_linear_vel = linear;
    ros2_angular_vel = angular;
    ros2_last_cmd_time = millis();
    ros2_connected = true;
    
    Serial.print("📡 ROS2 cmd_vel recibido: linear=");
    Serial.print(linear, 3);
    Serial.print(" m/s, angular=");
    Serial.print(angular, 3);
    Serial.println(" rad/s");
    
    #ifdef ENABLE_HALL_SENSORS
      if (currentRobotState != STATE_HABILITADO) {
        Serial.println("⚠️ Habilitando robot para movimiento...");
        setStateHabilitado();
        delay(100);
      }
      
      float v_meas = 0.0;
      float w_meas = 0.0;
      int pwm_left = 0, pwm_right = 0;
      pid_control_velocity(linear, angular, v_meas, w_meas, pwm_left, pwm_right);
      
      Serial.print("⚙️ PWM calculado: Left=");
      Serial.print(pwm_left);
      Serial.print(" Right=");
      Serial.println(pwm_right);
      
      bool dir_left = (linear >= 0);
      bool dir_right = (linear >= 0);
      
      setLeftMotor(pwm_left, dir_left);
      setRightMotor(pwm_right, dir_right);  // dir=true→adelante, la funcion maneja polaridad fisica interna

      // ── Anti-stición: registrar PWM base para el monitor ───────────────
      as_base_left  = pwm_left;
      as_base_right = pwm_right;
      as_dir_left   = dir_left;
      as_dir_right  = dir_right;
      as_cmd_ms     = millis();
      if (pwm_left  == 0) { as_boost_left  = 0; }
      if (pwm_right == 0) { as_boost_right = 0; }
      // ─────────────────────────────────────────────────────────────────

      Serial.print("🚀 Aplicando: Left PWM=");
      Serial.print(pwm_left);
      Serial.print(" dir=");
      Serial.print(dir_left ? "FWD" : "BWD");
      Serial.print(", Right PWM=");
      Serial.print(pwm_right);
      Serial.print(" dir=");
      Serial.println(!dir_right ? "FWD" : "BWD");
    #endif
  }
}

void ros2_processEncoderRequest() {
  #ifdef ENABLE_HALL_SENSORS
    noInterrupts();
    int32_t sL = leftHallSigned;
    int32_t sR = rightHallSigned;
    interrupts();
    ros2_port->print("e ");
    ros2_port->print(sL);
    ros2_port->print(" ");
    ros2_port->println(sR);
  #else
    ros2_port->println("e 0 0");
  #endif
}

void ros2_processOdomRequest() {
  #ifdef ENABLE_ODOMETRY
    ros2_port->print("o ");
    ros2_port->print(odom_getX(), 4);     ros2_port->print(" ");
    ros2_port->print(odom_getY(), 4);     ros2_port->print(" ");
    ros2_port->print(odom_getTheta(), 4); ros2_port->print(" ");
    ros2_port->print(odom_getV(), 4);     ros2_port->print(" ");
    ros2_port->println(odom_getW(), 4);
  #else
    ros2_port->println("o 0 0 0 0 0");
  #endif
}

void ros2_processEncoderReset() {
  #ifdef ENABLE_HALL_SENSORS
    resetHallCounters();
    noInterrupts();
    leftHallSigned  = 0;
    rightHallSigned = 0;
    interrupts();
    #ifdef ENABLE_ODOMETRY
      odom_reset();
    #endif
    ros2_port->println("r OK");
  #else
    ros2_port->println("r FAIL");
  #endif
}

void ros2_processStatusRequest() {
  ros2_port->print("s ");
  ros2_port->println(ros2_status);
}

void ros2_processImuRequest() {
  #ifdef ENABLE_MPU9250
    // Convertir a unidades SI (arduino_bridge.py espera rad/s y m/s²)
    float gz_rad_s = mpu_gyroZ_f * (PI / 180.0f);   // °/s → rad/s
    float ax_m_s2  = mpu_accelX_f * 9.80665f;        // g   → m/s²
    ros2_port->print("i ");
    ros2_port->print(gz_rad_s, 5);
    ros2_port->print(" ");
    ros2_port->println(ax_m_s2, 5);
  #else
    ros2_port->println("i 0.00000 0.00000");
  #endif
}

void ros2_processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.length() == 0) return;
  
  char command = cmd.charAt(0);
  
  switch (command) {
    case 'v':
      ros2_processCmdVel(cmd);
      break;

    case 'o':
      ros2_processOdomRequest();
      break;

    case 'c':
      pid_calibrate();
      break;
      
    case 'e':
      ros2_processEncoderRequest();
      break;
      
    case 'r':
      ros2_processEncoderReset();
      break;
      
    case 's':
      ros2_processStatusRequest();
      break;

    case 'i':
      ros2_processImuRequest();
      break;
      
    default:
      Serial.print("? Unknown ROS2 command: ");
      Serial.println(cmd);
      break;
  }
}

//===========================================================================
//==================== DETECCIÓN DE COMANDOS ROS2 ===========================
//===========================================================================

bool ros2_tryProcessCommand(String cmd) {
  cmd.trim();
  
  if (cmd.length() == 0) return false;
  
  char first_char = cmd.charAt(0);
  
  // Verificar si es comando ROS2 (empieza con v, e, r, s, c)
  if (cmd.length() >= 1) {
    if ((first_char == 'v') && (cmd.length() == 1 || cmd.charAt(1) == ' ')) {
      ros2_processCommand(cmd);
      return true;
    }
    else if ((first_char == 'o') && cmd.length() == 1) {
      ros2_processCommand(cmd);
      return true;
    }
    else if ((first_char == 'e' || first_char == 'r' || first_char == 's' || first_char == 'c' || first_char == 'i') && cmd.length() == 1) {
      ros2_processCommand(cmd);
      return true;
    }
  }
  
  return false;
}

//===========================================================================
//==================== ANTI-STICIÓN: boost PWM si rueda parada =============
//===========================================================================

/**
 * ros2_antiStiction()
 * Ejecuta cada AS_PERIOD_MS. Si una rueda tiene PWM > 0 pero no gira
 * (velocidad Hall < AS_MIN_SPEED) y han pasado AS_START_DELAY_MS desde
 * el último cmd_vel, incrementa el PWM en AS_BOOST_STEP (máx AS_MAX_BOOST).
 * En cuanto la rueda gira, resetea el boost al PWM base.
 * Aplicado a ambas ruedas de forma independiente.
 * Llamar desde ros2_update().
 */
#ifdef ENABLE_HALL_SENSORS
void ros2_antiStiction() {
  if (as_base_left == 0 && as_base_right == 0) return;

  unsigned long now = millis();
  if (now - as_last_ms < AS_PERIOD_MS) return;
  as_last_ms = now;

  // No activar hasta que pase el retardo de arranque
  if (now - as_cmd_ms < AS_START_DELAY_MS) return;

  // ── Motor izquierdo ──────────────────────────────────────────────
  if (as_base_left > 0) {
    if (currentSpeedLeftHall < AS_MIN_SPEED) {
      as_boost_left = min(as_boost_left + AS_BOOST_STEP, AS_MAX_BOOST);
    } else {
      as_boost_left = 0;  // girando: resetear boost
    }
    int eff_left = constrain(as_base_left + as_boost_left, 0, (int)MAX_PWM_VALUE);
    setLeftMotor(eff_left, as_dir_left);
  }

  // ── Motor derecho ──────────────────────────────────────────────
  if (as_base_right > 0) {
    if (currentSpeedRightHall < AS_MIN_SPEED) {
      as_boost_right = min(as_boost_right + AS_BOOST_STEP, AS_MAX_BOOST);
    } else {
      as_boost_right = 0;  // girando: resetear boost
    }
    int eff_right = constrain(as_base_right + as_boost_right, 0, (int)MAX_PWM_VALUE);
    setRightMotor(eff_right, as_dir_right);
  }
}
#endif

//===========================================================================
//==================== UPDATE LOOP ==========================================
//===========================================================================

void ros2_update() {
  // NOTA: odom_update() se llama desde updateHallSpeeds() en Hall_Sensors.h

  // Timeout seguridad: parar motores si no llega cmd_vel
  if (ros2_connected && (millis() - ros2_last_cmd_time > ROS2_CMD_TIMEOUT)) {
    ros2_linear_vel  = 0.0;
    ros2_angular_vel = 0.0;
    ros2_connected   = false;
    as_base_left  = 0; as_base_right  = 0;   // cancelar anti-stición
    as_boost_left = 0; as_boost_right = 0;
    setLeftMotor(0, true);
    setRightMotor(0, true);
    ros2_status = ROS2_STATUS_WARNING;
  }

  // Anti-stición: boost si rueda no arranca
  #ifdef ENABLE_HALL_SENSORS
    ros2_antiStiction();
  #endif
}

// Cambio manual de puerto — llamado desde Serial_Command_Processor
void ros2_useUSB() {
  ros2_port     = &Serial;
  ros2_using_bt = false;
  Serial.println(F("[ROS2] Puerto activo: USB"));
}

#ifdef ENABLE_BLUETOOTH
void ros2_useBT() {
  if (!bt_connected) {
    Serial.println(F("[ROS2] ERROR: BT no conectado. Empareja el HC-05 primero."));
    return;
  }
  ros2_port     = &BT_SERIAL;
  ros2_using_bt = true;
  Serial.println(F("[ROS2] Puerto activo: Bluetooth"));
}
#endif

bool ros2_isActive() {
  return ros2_connected;
}

float ros2_getLinearVel() {
  return ros2_linear_vel;
}

float ros2_getAngularVel() {
  return ros2_angular_vel;
}

#endif // ENABLE_ROS2_BRIDGE

#endif // ROS2_BRIDGE_H
