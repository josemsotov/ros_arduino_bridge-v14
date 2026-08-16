/**
 * ============================================================================
 * MOTOR-INTERFACE-V-13 - ROS2 BRIDGE MODULE
 * Smart Golf Trolley - Interfaz con ROS2 (Raspberry Pi 5) - VERSIÓN LIMPIA
 * ============================================================================
 * 
 * PROTOCOLO:
 * Arduino <- Raspberry Pi:
 *   v <linear_vel> <angular_vel>  : Comando de velocidad (m/s, rad/s)
 *   e                              : Request encoder counts
 *   r                              : Reset encoders
 *   s                              : Request status
 *   c                              : Calibración PID
 *   
 * Arduino -> Raspberry Pi:
 *   e <left> <right>               : Encoder counts
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

#define ROS2_CMD_TIMEOUT 1000  // Timeout para comandos de velocidad (ms)
#define ROS2_MAX_LINEAR_VEL 1.0   // m/s
#define ROS2_MAX_ANGULAR_VEL 2.0  // rad/s
#define ROS2_BUFFER_SIZE 128

//===========================================================================
//==================== VARIABLES GLOBALES ROS2 ==============================
//===========================================================================

// ── Integradores per-rueda con anti-windup ─────────────────────────────────
// Clamped en MAX_PWM_VALUE/100 para evitar windup cuando el motor está bloqueado.
static float integral_left  = 0.0f;
static float integral_right = 0.0f;
static unsigned long pid_per_wheel_last_time = 0;

void pid_per_wheel_reset() {
  integral_left  = 0.0f;
  integral_right = 0.0f;
  pid_per_wheel_last_time = 0;
  // Limpiar velocidades Hall stale para que FF domine al arrancar
  currentSpeedLeftHall  = 0.0f;
  currentSpeedRightHall = 0.0f;
}

bool ros2_connected = false;
unsigned long ros2_last_cmd_time = 0;

float ros2_linear_vel = 0.0;   // m/s
float ros2_angular_vel = 0.0;  // rad/s

// Continuous low-speed output. Calibration on 2026-08-04 confirmed both
// wheels rotate continuously from PWM=10; temporal pulsing is unnecessary.
static float ros2_pwm_demand_left = 0.0f;
static float ros2_pwm_demand_right = 0.0f;
static bool ros2_pwm_dir_left = true;
static bool ros2_pwm_dir_right = true;
static int ros2_pwm_applied_left = -1;
static int ros2_pwm_applied_right = -1;

#if defined(ENABLE_MPU9250) && defined(ENABLE_MPU_HEADING_CONTROL)
// Starts disabled until gyro sign is confirmed once on the physical robot.
// Enable at runtime with `hc on`; the setting will become the default after
// the protected bench calibration.
static bool heading_control_enabled = false;
static bool heading_hold_active = false;
static float heading_reference_deg = 0.0f;
static float heading_last_error_deg = 0.0f;
static float heading_last_w_raw = 0.0f;
static float heading_last_w_meas = 0.0f;
static float heading_last_w_output = 0.0f;
static float heading_gyro_bias_rad_s = HEADING_GYRO_BIAS_RAD_S;
static float heading_gyro_alpha = HEADING_GYRO_FILTER_ALPHA;
static float heading_gyro_noise_rad_s = HEADING_GYRO_NOISE_RAD_S;
static float heading_gyro_filtered_rad_s = 0.0f;

float heading_wrap_deg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float heading_control_yaw_deg() {
  return HEADING_GYRO_SIGN * mpu_getYaw();
}

void heading_control_reset() {
  heading_hold_active = false;
  heading_reference_deg = heading_control_yaw_deg();
  heading_last_error_deg = 0.0f;
  heading_last_w_raw = 0.0f;
  heading_last_w_meas = 0.0f;
  heading_last_w_output = 0.0f;
  heading_gyro_filtered_rad_s = 0.0f;
}

float heading_filter_gyro(float raw_rad_s) {
  float unbiased = raw_rad_s - heading_gyro_bias_rad_s;
  float alpha = constrain(heading_gyro_alpha, 0.02f, 1.0f);
  heading_gyro_filtered_rad_s +=
    alpha * (unbiased - heading_gyro_filtered_rad_s);
  if (fabsf(heading_gyro_filtered_rad_s) <= heading_gyro_noise_rad_s) {
    return 0.0f;
  }
  return heading_gyro_filtered_rad_s;
}

float heading_control_apply(float linear, float angular_request) {
  if (!mpu_isReady()) {
    heading_hold_active = false;
    heading_last_w_output = angular_request;
    return angular_request;
  }

  const float gyro_raw_rad_s = HEADING_GYRO_SIGN * mpu_getGyroZ() * (PI / 180.0f);
  heading_last_w_raw = gyro_raw_rad_s;
  const float gyro_rad_s = heading_filter_gyro(gyro_raw_rad_s);
  heading_last_w_meas = gyro_rad_s;
  if (!heading_control_enabled) {
    heading_hold_active = false;
    heading_last_w_output = angular_request;
    return angular_request;
  }

  // Intentional steering has priority; close its angular-rate loop.
  if (fabsf(angular_request) >= HEADING_ANGULAR_DEADBAND_RAD_S) {
    heading_hold_active = false;
    float rate_error = angular_request - gyro_rad_s;
    float correction = constrain(HEADING_RATE_KP * rate_error,
      -HEADING_MAX_CORRECTION_RAD_S, HEADING_MAX_CORRECTION_RAD_S);
    heading_last_error_deg = 0.0f;
    heading_last_w_output = constrain(angular_request + correction,
      -ROS2_MAX_ANGULAR_VEL, ROS2_MAX_ANGULAR_VEL);
    return heading_last_w_output;
  }

  // At rest, do not fight gyro noise. Capture yaw when translation starts.
  if (fabsf(linear) < HEADING_LINEAR_ACTIVE_M_S) {
    heading_control_reset();
    return angular_request;
  }
  if (!heading_hold_active) {
    heading_reference_deg = heading_control_yaw_deg();
    heading_hold_active = true;
  }

  float error_deg = heading_wrap_deg(
    heading_reference_deg - heading_control_yaw_deg());
  if (fabsf(error_deg) < HEADING_ERROR_DEADBAND_DEG) error_deg = 0.0f;
  heading_last_error_deg = error_deg;
  float correction = HEADING_HOLD_KP * error_deg - HEADING_RATE_KP * gyro_rad_s;
  heading_last_w_output = constrain(correction,
    -HEADING_MAX_CORRECTION_RAD_S, HEADING_MAX_CORRECTION_RAD_S);
  return heading_last_w_output;
}
#endif

int ros2_time_proportioned_pwm(float demand, int working_pwm, unsigned long phase) {
  (void)phase;
  if (demand <= 0.0f) return 0;
  return constrain((int)roundf(demand), working_pwm, MAX_PWM_VALUE);
}

void ros2_apply_pwm_demands() {
  unsigned long phase = 0;
  int next_left = ros2_time_proportioned_pwm(
    ros2_pwm_demand_left, MIN_PWM_VALUE, phase);
  int next_right = ros2_time_proportioned_pwm(
    ros2_pwm_demand_right, MIN_PWM_RIGHT_WORKING, phase);
  if (next_left != ros2_pwm_applied_left ||
      ros2_pwm_dir_left != leftMotor.direction) {
    setLeftMotor(next_left, ros2_pwm_dir_left);
    ros2_pwm_applied_left = next_left;
  }
  bool physical_right_dir = !ros2_pwm_dir_right;
  if (next_right != ros2_pwm_applied_right ||
      physical_right_dir != rightMotor.direction) {
    setRightMotor(next_right, physical_right_dir);
    ros2_pwm_applied_right = next_right;
  }
}

void ros2_clear_pwm_demands() {
  ros2_pwm_demand_left = 0.0f;
  ros2_pwm_demand_right = 0.0f;
  ros2_pwm_applied_left = -1;
  ros2_pwm_applied_right = -1;
#if defined(ENABLE_MPU9250) && defined(ENABLE_MPU_HEADING_CONTROL)
  heading_control_reset();
#endif
}

// Extern al Balance_Controller (incluido después): base de velocidad del usuario
// y flag para suprimir telemetría cuando el balance llama ros2_processCmdVel
#if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)
extern float bal_base_linear;
extern float bal_base_angular;
extern bool  bal_suppress_telemetry;
#endif

char ros2_buffer[ROS2_BUFFER_SIZE];
int ros2_buffer_index = 0;

enum ROS2_Status {
  ROS2_STATUS_OK = 0,
  ROS2_STATUS_ERROR = 1,
  ROS2_STATUS_WARNING = 2
};

ROS2_Status ros2_status = ROS2_STATUS_OK;

//===========================================================================
//==================== FUNCIONES DE INICIALIZACIÓN ==========================
//===========================================================================

void ros2_initialize() {
  ros2_connected = false;
  ros2_linear_vel = 0.0;
  ros2_angular_vel = 0.0;
  ros2_buffer_index = 0;
  ros2_last_cmd_time = millis();
  ros2_status = ROS2_STATUS_OK;
  
  Serial.println("\n🤖 ROS2 BRIDGE INICIALIZADO");
  Serial.println("   Esperando conexión de Raspberry Pi...");
  Serial.println("   Protocolo: Comandos de texto serial");
  Serial.println("   Baud rate: 115200");
  Serial.println("");
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

    // Actualizar velocidad base para el balance anti-caída
    // Solo cuando el comando viene del USUARIO (bal_suppress_telemetry=false).
    // Cuando el balance llama esta función internamente (bal_suppress_telemetry=true)
    // NO se sobreescribe bal_base_linear para evitar acumulación de correcciones:
    //   sin este guard: bal_base = (base+cor) → siguiente ciclo usa (base+cor)+cor = base+2*cor
    //   con este guard: bal_base siempre refleja la intención del usuario, no el output del balance
    #if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)
    if (!bal_suppress_telemetry) {
      bal_base_linear  = linear;
      bal_base_angular = angular;
    }
    #endif
    
    // Telemetría compacta: se emite al final del procesamiento (ver línea T)
    
    #ifdef ENABLE_HALL_SENSORS
      if (currentRobotState != STATE_HABILITADO) {
        Serial.println("⚠️ Habilitando robot para movimiento...");
        setStateHabilitado();
        delay(100);
      }

      // ── PARADA DIRECTA (sin PID) ──────────────────────────────────────────
      // Cuando setpoint es cero no usar PID: los Hall son sin signo y no se
      // puede distinguir si el robot frena o aun se mueve. Detener directo
      // evita el bucle de retroalimentacion positiva.
      if (fabsf(linear) < 0.01f && fabsf(angular) < 0.01f) {
        ros2_clear_pwm_demands();
        setLeftMotor(0, true);
        setRightMotor(0, false);
        pid_reset_velocity();
        pid_per_wheel_reset();
        #if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)
        if (!bal_suppress_telemetry)
        #endif
        {
          Serial.print("T lin=0.000 ang=0.000 Lpwm=0 Rpwm=0 Lrpm=0 Rrpm=0 Ld=F Rd=F");
          #ifdef ENABLE_CURRENT_SENSORS
          Serial.print(F(" LmA=0.00 RmA=0.00"));
          #endif
          Serial.println();
        }
        return;
      }

      // ── CINEMATICA: signo de velocidad por rueda ──────────────────────────
      // v_left  = v + w*L/2   (giro CCW positivo → rueda izq mas rapida)
      // v_right = v - w*L/2   (giro CCW positivo → rueda der mas lenta)
      float angular_controlled = angular;
      #if defined(ENABLE_MPU9250) && defined(ENABLE_MPU_HEADING_CONTROL)
      angular_controlled = heading_control_apply(linear, angular);
      #endif

      const float wb2 = WHEEL_BASE_DISTANCE_M / 2.0f;
      float v_left_raw  = linear + angular_controlled * wb2;
      float v_right_raw = linear - angular_controlled * wb2;
      bool  cmd_dir_left  = (v_left_raw  >= 0.0f);  // direccion comandada izq
      bool  cmd_dir_right = (v_right_raw >= 0.0f);  // direccion comandada der

      // ── CAMBIO DE DIRECCIÓN: stop previo al driver ────────────────────────
      // ZS-X11H (BTS7960) ignora cambio de DIR si el PWM está activo.
      // Al detectar inversión: PWM=0 por 5ms + limpiar Hall stale + limpiar integral.
      // CRÍTICO: usar motor_pwm_write(0), NO analogWrite(0).
      // analogWrite(pin,0) llama turnOffPWM() → limpia COM5A1/COM5C1 en TCCR5A
      // → pines 44/46 quedan como GPIO LOW y no emiten PWM aunque OCR sea >0.
      static bool prev_cmd_dir_left  = true;
      static bool prev_cmd_dir_right = true;
      if (cmd_dir_left != prev_cmd_dir_left) {
        motor_pwm_write(PWM_LEFT_MOTOR, 0);
        currentSpeedLeftHall = 0.0f;
        integral_left = 0.0f;
        delay(5);
      }
      if (cmd_dir_right != prev_cmd_dir_right) {
        motor_pwm_write(PWM_RIGHT_MOTOR, 0);
        currentSpeedRightHall = 0.0f;
        integral_right = 0.0f;
        delay(5);
      }
      prev_cmd_dir_left  = cmd_dir_left;
      prev_cmd_dir_right = cmd_dir_right;

      // ── CONTROL FF PURO POR MOTOR + SPEED-MATCHING ───────────────────────
      // 2026-07-07: Reemplaza PI+anti-stall que oscilaba porque
      //   MIN_PWM_RIGHT_WORKING(60) > FF*v para todo el rango operativo (0-0.5 m/s).
      // Nuevo diseño:
      //   1. FF por motor y direccion (calibrado con comandos x/xl/ff)
      //   2. Direccion tomada de cmd_dir (no de signo PI — arreglaba backward erratico)
      //   3. Speed-matching suave cuando ambas ruedas tienen datos Hall validos

      // Ganancias FF por direccion (calibradas; se actualizan con comando ff)
      float ff_l = cmd_dir_left  ? FF_LEFT_GAIN  : FF_LEFT_BWD;
      float ff_r = cmd_dir_right ? FF_RIGHT_GAIN : FF_RIGHT_BWD;

      bool l_moving = (fabsf(v_left_raw)  > 0.01f);
      bool r_moving = (fabsf(v_right_raw) > 0.01f);

      float demand_left = l_moving
        ? constrain(fabsf(v_left_raw) * ff_l, 0.0f, (float)MAX_PWM_VALUE) : 0.0f;
      float demand_right = r_moving
        ? constrain(fabsf(v_right_raw) * ff_r, 0.0f, (float)MAX_PWM_VALUE) : 0.0f;
      int abs_left = (int)roundf(demand_left);
      int abs_right = (int)roundf(demand_right);

      // Direccion directa de cinematica (evita ambiguedad de signo del PID)
      bool dir_left  = cmd_dir_left;
      bool dir_right = cmd_dir_right;

      // Stop-before-reverse
      if (abs_left  > 0 && dir_left   != leftMotor.direction)  { motor_pwm_write(PWM_LEFT_MOTOR,  0); delayMicroseconds(2000); }
      if (abs_right > 0 && !dir_right != rightMotor.direction) { motor_pwm_write(PWM_RIGHT_MOTOR, 0); delayMicroseconds(2000); }

      // Speed-matching Hall: corrige asimetria RPM en linea recta
      if (cmd_dir_left == cmd_dir_right &&
          fabsf(angular_controlled) < 0.05f &&
          l_moving && r_moving &&
          currentSpeedLeftHall  > 3.0f &&
          currentSpeedRightHall > 3.0f &&
          demand_left >= MIN_PWM_VALUE &&
          demand_right >= MIN_PWM_RIGHT_WORKING) {
        float rpm_error = currentSpeedRightHall - currentSpeedLeftHall;
        int speed_trim = (int)constrain(
          rpm_error * SPEED_MATCH_KP_PWM_PER_RPM,
          -SPEED_MATCH_MAX_PWM, SPEED_MATCH_MAX_PWM);
        abs_left  = constrain(abs_left  + speed_trim, MIN_PWM_VALUE,         MAX_PWM_VALUE);
        abs_right = constrain(abs_right - speed_trim, MIN_PWM_RIGHT_WORKING, MAX_PWM_VALUE);
        demand_left = (float)abs_left;
        demand_right = (float)abs_right;
      }

      ros2_pwm_demand_left = demand_left;
      ros2_pwm_demand_right = demand_right;
      ros2_pwm_dir_left = dir_left;
      ros2_pwm_dir_right = dir_right;
      ros2_apply_pwm_demands();
      abs_left = max(0, ros2_pwm_applied_left);
      abs_right = max(0, ros2_pwm_applied_right);

      // ── Telemetría compacta ────────────────────────────────────────────────
      // Suprimida cuando el balance anti-caída llama esta función a 50 Hz
      #if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)
      if (!bal_suppress_telemetry)
      #endif
      {
        Serial.print("T lin="); Serial.print(linear, 3);
        Serial.print(" ang=");  Serial.print(angular, 3);
#if defined(ENABLE_MPU9250) && defined(ENABLE_MPU_HEADING_CONTROL)
        Serial.print(" hcw=");  Serial.print(angular_controlled, 3);
        Serial.print(" hce=");  Serial.print(heading_last_error_deg, 1);
        Serial.print(" gyr=");  Serial.print(heading_last_w_raw, 5);
        Serial.print(" gyz=");  Serial.print(heading_last_w_meas, 3);
#endif
        Serial.print(" Lpwm="); Serial.print(abs_left);
        Serial.print(" Rpwm="); Serial.print(abs_right);
        Serial.print(" Lrpm="); Serial.print((int)currentSpeedLeftHall);
        Serial.print(" Rrpm="); Serial.print((int)currentSpeedRightHall);
        Serial.print(" Ld=");   Serial.print(dir_left  ? "F" : "B");
        Serial.print(" Rd=");   Serial.print(dir_right ? "F" : "B");
        #ifdef ENABLE_CURRENT_SENSORS
        Serial.print(F(" LmA=")); Serial.print(current_left_A,  2);
        Serial.print(F(" RmA=")); Serial.print(current_right_A, 2);
        #endif
        Serial.println();
      }
    #endif
  }
}

void ros2_processEncoderRequest() {
  #ifdef ENABLE_HALL_SENSORS
    // Usar contadores acumulativos (no se resetean cada 100ms como leftHallCount)
    noInterrupts();
    uint32_t l = leftHallTotal;
    uint32_t r = rightHallTotal;
    interrupts();
    Serial.print("e ");
    Serial.print(l);
    Serial.print(" ");
    Serial.println(r);
  #else
    Serial.println("e 0 0");
  #endif
}

void ros2_processEncoderReset() {
  #ifdef ENABLE_HALL_SENSORS
    resetHallCounters();
    Serial.println("✅ Encoders reseteados");
    Serial.println("r OK");
  #else
    Serial.println("r FAIL");
  #endif
}

void ros2_processStatusRequest() {
  Serial.print("s ");
  Serial.println(ros2_status);
}

// Lectura independiente: "o <pulsos_izq> <pulsos_der>".
void ros2_processOptoRequest() {
  #ifdef ENABLE_OPTO_ENCODERS
    noInterrupts();
    uint32_t l = leftOptoCount;
    uint32_t r = rightOptoCount;
    interrupts();
    Serial.print("o ");
    Serial.print(l);
    Serial.print(" ");
    Serial.println(r);
  #else
    Serial.println("o 0 0");
  #endif
}

#if defined(ENABLE_HALL_SENSORS) && defined(ENABLE_OPTO_ENCODERS)
enum EncoderHealthState : uint8_t {
  ENC_HEALTH_IDLE = 0,
  ENC_HEALTH_OK = 1,
  ENC_HEALTH_LOW = 2,
  ENC_HEALTH_HIGH = 3
};

static EncoderHealthState encoder_health_left = ENC_HEALTH_IDLE;
static EncoderHealthState encoder_health_right = ENC_HEALTH_IDLE;
static EncoderHealthState encoder_candidate_left = ENC_HEALTH_IDLE;
static EncoderHealthState encoder_candidate_right = ENC_HEALTH_IDLE;
static uint8_t encoder_streak_left = 0;
static uint8_t encoder_streak_right = 0;
static bool encoder_protection_enabled = false;
static bool encoder_protection_latched = false;
static uint32_t encoder_prev_hall_left = 0;
static uint32_t encoder_prev_hall_right = 0;
static uint32_t encoder_prev_opto_left = 0;
static uint32_t encoder_prev_opto_right = 0;
static uint16_t encoder_last_hall_left = 0;
static uint16_t encoder_last_hall_right = 0;
static uint16_t encoder_last_opto_left = 0;
static uint16_t encoder_last_opto_right = 0;
static unsigned long encoder_health_last_ms = 0;

EncoderHealthState encoder_classify_window(uint32_t hall, uint32_t opto) {
  if (hall < 3) return ENC_HEALTH_IDLE;
  const uint32_t measured_scaled = opto * PPR_HALL_SENSORS * 100UL;
  const uint32_t expected_scaled = hall * PPR_OPTO_ENCODERS;
  if (measured_scaled < expected_scaled * 80UL) return ENC_HEALTH_LOW;
  if (measured_scaled > expected_scaled * 120UL) return ENC_HEALTH_HIGH;
  return ENC_HEALTH_OK;
}

const __FlashStringHelper* encoder_health_name(EncoderHealthState state) {
  switch (state) {
    case ENC_HEALTH_OK: return F("OK");
    case ENC_HEALTH_LOW: return F("LOW");
    case ENC_HEALTH_HIGH: return F("HIGH");
    default: return F("IDLE");
  }
}

void encoder_update_confirmed_state(EncoderHealthState sample,
                                    EncoderHealthState &confirmed,
                                    EncoderHealthState &candidate,
                                    uint8_t &streak) {
  if (sample == ENC_HEALTH_OK || sample == ENC_HEALTH_IDLE) {
    confirmed = sample;
    candidate = sample;
    streak = 0;
    return;
  }
  if (sample != candidate) {
    candidate = sample;
    streak = 1;
  } else if (streak < 255) {
    streak++;
  }
  if (streak >= 3) confirmed = sample;
}

void encoder_supervisor_update() {
  unsigned long now = millis();
  if (now - encoder_health_last_ms < 500UL) return;
  encoder_health_last_ms = now;

  noInterrupts();
  uint32_t hl = leftHallTotal;
  uint32_t hr = rightHallTotal;
  uint32_t ol = leftOptoCount;
  uint32_t oright = rightOptoCount;
  interrupts();

  if (hl < encoder_prev_hall_left || ol < encoder_prev_opto_left) {
    encoder_prev_hall_left = hl;
    encoder_prev_opto_left = ol;
  }
  if (hr < encoder_prev_hall_right || oright < encoder_prev_opto_right) {
    encoder_prev_hall_right = hr;
    encoder_prev_opto_right = oright;
  }
  uint32_t dhl = hl - encoder_prev_hall_left;
  uint32_t dhr = hr - encoder_prev_hall_right;
  uint32_t dol = ol - encoder_prev_opto_left;
  uint32_t dor = oright - encoder_prev_opto_right;
  encoder_prev_hall_left = hl;
  encoder_prev_hall_right = hr;
  encoder_prev_opto_left = ol;
  encoder_prev_opto_right = oright;
  encoder_last_hall_left = min(dhl, 65535UL);
  encoder_last_hall_right = min(dhr, 65535UL);
  encoder_last_opto_left = min(dol, 65535UL);
  encoder_last_opto_right = min(dor, 65535UL);

  EncoderHealthState left_sample = encoder_classify_window(dhl, dol);
  EncoderHealthState right_sample = encoder_classify_window(dhr, dor);
  encoder_update_confirmed_state(left_sample, encoder_health_left,
    encoder_candidate_left, encoder_streak_left);
  encoder_update_confirmed_state(right_sample, encoder_health_right,
    encoder_candidate_right, encoder_streak_right);

  if (encoder_protection_enabled && !encoder_protection_latched &&
      (encoder_health_left == ENC_HEALTH_LOW || encoder_health_left == ENC_HEALTH_HIGH ||
       encoder_health_right == ENC_HEALTH_LOW || encoder_health_right == ENC_HEALTH_HIGH)) {
    encoder_protection_latched = true;
    ros2_connected = false;
    ros2_clear_pwm_demands();
    setLeftMotor(0, true);
    setRightMotor(0, false);
    setStateInhabilitado();
    Serial.print(F("n TRIP L=")); Serial.print(encoder_health_name(encoder_health_left));
    Serial.print(F(" R=")); Serial.println(encoder_health_name(encoder_health_right));
  }
}

void encoder_print_health(const __FlashStringHelper* prefix,
                          uint32_t hl, uint32_t hr,
                          uint32_t ol, uint32_t oright) {
  Serial.print(prefix);
  Serial.print(F(" L=")); Serial.print(encoder_health_name(encoder_classify_window(hl, ol)));
  Serial.print(F(" R=")); Serial.print(encoder_health_name(encoder_classify_window(hr, oright)));
  Serial.print(F(" HL=")); Serial.print(hl);
  Serial.print(F(" HR=")); Serial.print(hr);
  Serial.print(F(" OL=")); Serial.print(ol);
  Serial.print(F(" OR=")); Serial.println(oright);
}
#endif

// Forward declarations de módulos que se incluyen después
#if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)
void hoverboard_processCommand(String args);
#endif

void ros2_processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.length() == 0) return;
  
  char command = cmd.charAt(0);
  
  switch (command) {
    case 'v':
      ros2_processCmdVel(cmd);
      break;

    case 'n': {
      #if defined(ENABLE_HALL_SENSORS) && defined(ENABLE_OPTO_ENCODERS)
      String args = cmd.substring(1);
      args.trim();
      if (args == "selftest") {
        bool pass =
          encoder_classify_window(0, 0) == ENC_HEALTH_IDLE &&
          encoder_classify_window(45, 60) == ENC_HEALTH_OK &&
          encoder_classify_window(45, 30) == ENC_HEALTH_LOW &&
          encoder_classify_window(45, 90) == ENC_HEALTH_HIGH;
        Serial.println(pass ? F("n SELFTEST PASS") : F("n SELFTEST FAIL"));
      } else if (args == "check") {
        noInterrupts();
        uint32_t hl = leftHallTotal, hr = rightHallTotal;
        uint32_t ol = leftOptoCount, oright = rightOptoCount;
        interrupts();
        encoder_print_health(F("n CHECK"), hl, hr, ol, oright);
      } else if (args == "protect on") {
        encoder_protection_enabled = true;
        encoder_protection_latched = false;
        encoder_health_left = ENC_HEALTH_IDLE;
        encoder_health_right = ENC_HEALTH_IDLE;
        encoder_streak_left = 0;
        encoder_streak_right = 0;
        Serial.println(F("n PROTECT ON"));
      } else if (args == "protect off") {
        encoder_protection_enabled = false;
        encoder_protection_latched = false;
        Serial.println(F("n PROTECT OFF"));
      } else if (args == "protect reset") {
        encoder_protection_latched = false;
        encoder_health_left = ENC_HEALTH_IDLE;
        encoder_health_right = ENC_HEALTH_IDLE;
        encoder_candidate_left = ENC_HEALTH_IDLE;
        encoder_candidate_right = ENC_HEALTH_IDLE;
        encoder_streak_left = 0;
        encoder_streak_right = 0;
        Serial.println(F("n PROTECT RESET"));
      } else {
        Serial.print(F("n STATUS L=")); Serial.print(encoder_health_name(encoder_health_left));
        Serial.print(F(" R=")); Serial.print(encoder_health_name(encoder_health_right));
        Serial.print(F(" protect=")); Serial.print(encoder_protection_enabled ? 1 : 0);
        Serial.print(F(" latched=")); Serial.print(encoder_protection_latched ? 1 : 0);
        Serial.print(F(" HL=")); Serial.print(encoder_last_hall_left);
        Serial.print(F(" HR=")); Serial.print(encoder_last_hall_right);
        Serial.print(F(" OL=")); Serial.print(encoder_last_opto_left);
        Serial.print(F(" OR=")); Serial.println(encoder_last_opto_right);
      }
      #else
      Serial.println(F("n DISABLED"));
      #endif
      break;
    }

#if defined(ENABLE_MPU9250) && defined(ENABLE_MPU_HEADING_CONTROL)
    case 'h': {
      // Preserve the existing hoverboard command family.
      #if defined(ENABLE_HOVERBOARD_MODE)
      if (cmd.startsWith("hb")) {
        String hb_args = cmd.length() > 3 ? cmd.substring(3) : "";
        hoverboard_processCommand(hb_args);
        break;
      }
      #endif
      // hc on|off|reset|stat|filter
      if (!cmd.startsWith("hc")) break;
      String args = cmd.substring(2);
      args.trim();
      if (args == "on") {
        heading_control_enabled = true;
        heading_control_reset();
      } else if (args == "off") {
        heading_control_enabled = false;
        heading_control_reset();
      } else if (args == "reset") {
        heading_control_reset();
      } else if (args.startsWith("filter ")) {
        String values = args.substring(7);
        int sp1 = values.indexOf(' ');
        int sp2 = values.indexOf(' ', sp1 + 1);
        if (sp1 > 0 && sp2 > sp1) {
          heading_gyro_bias_rad_s = values.substring(0, sp1).toFloat();
          heading_gyro_alpha = constrain(
            values.substring(sp1 + 1, sp2).toFloat(), 0.02f, 1.0f);
          heading_gyro_noise_rad_s = constrain(
            values.substring(sp2 + 1).toFloat(), 0.0f, 0.20f);
          heading_control_reset();
        }
      }
      Serial.print("hc enabled="); Serial.print(heading_control_enabled ? 1 : 0);
      Serial.print(" hold="); Serial.print(heading_hold_active ? 1 : 0);
      Serial.print(" ref="); Serial.print(heading_reference_deg, 2);
      Serial.print(" yaw="); Serial.print(heading_control_yaw_deg(), 2);
      Serial.print(" err="); Serial.print(heading_last_error_deg, 2);
      Serial.print(" raw="); Serial.print(
        HEADING_GYRO_SIGN * mpu_getGyroZ() * (PI / 180.0f), 5);
      Serial.print(" gyro="); Serial.print(heading_gyro_filtered_rad_s, 5);
      Serial.print(" bias="); Serial.print(heading_gyro_bias_rad_s, 5);
      Serial.print(" alpha="); Serial.print(heading_gyro_alpha, 3);
      Serial.print(" noise="); Serial.print(heading_gyro_noise_rad_s, 5);
      Serial.print(" out="); Serial.println(heading_last_w_output, 3);
      break;
    }
#endif

    case 'k': {
      // k <kp_v> [ki_v]  — cambia Kp_v y opcionalmente Ki_v en runtime
      // Responde: k OK Kp_v=<kp> Ki_v=<ki>
      int sp = cmd.indexOf(' ');
      if (sp > 0) {
        String params = cmd.substring(sp + 1);
        int sp2 = params.indexOf(' ');
        if (sp2 > 0) {
          Kp_v = params.substring(0, sp2).toFloat();
          Ki_v = params.substring(sp2 + 1).toFloat();
        } else {
          Kp_v = params.toFloat();
        }
        pid_per_wheel_reset();
        Serial.print("k OK Kp_v=");
        Serial.print(Kp_v, 4);
        Serial.print(" Ki_v=");
        Serial.println(Ki_v, 4);
      } else {
        Serial.print("k Kp_v=");
        Serial.print(Kp_v, 4);
        Serial.print(" Ki_v=");
        Serial.println(Ki_v, 4);
      }
      break;
    }
    case 'p': {
      // p <pwm>  — aplica PWM directo a ambos motores (FWD) sin PID
      // Util para medir ganancia de planta. Para 2s, luego para.
      // Responde: p OK pwm=<valor>
      int sp = cmd.indexOf(' ');
      if (sp > 0) {
        int rawPwm = constrain(cmd.substring(sp + 1).toInt(), 0, MAX_PWM_VALUE);
        if (currentRobotState != STATE_HABILITADO) { setStateHabilitado(); delay(50); }
        noInterrupts();
        leftHallTotal  = 0;
        rightHallTotal = 0;
        leftHallCount  = 0;
        rightHallCount = 0;
        interrupts();
        setLeftMotor(rawPwm, true);
        setRightMotor(rawPwm, false);  // derecho: DIR electrica invertida
        ros2_last_cmd_time = millis(); // evita que el timeout de 1s pare los motores
        Serial.print("p OK pwm=");
        Serial.println(rawPwm);
      } else {
        // sin argumento: detener motores
        setLeftMotor(0, true);
        setRightMotor(0, false);
        Serial.println("p STOP");
      }
      break;
    }
    case 'q': {
      // q <L|R> <pwm> - prueba individual continua de 1 s, limitada a
      // MAX_PWM_VALUE, bypass del minimo de trabajo para calibrarlo.
      int sp1 = cmd.indexOf(' ');
      int sp2 = cmd.indexOf(' ', sp1 + 1);
      if (sp1 < 0 || sp2 < 0) {
        Serial.println("q FAIL usage=q L|R pwm");
        break;
      }
      char side = cmd.charAt(sp1 + 1);
      int test_pwm = constrain(cmd.substring(sp2 + 1).toInt(), 0, MAX_PWM_VALUE);
      if (side != 'L' && side != 'R') {
        Serial.println("q FAIL side");
        break;
      }
      setLeftMotor(0, true);
      setRightMotor(0, false);
      if (currentRobotState != STATE_HABILITADO) {
        setStateHabilitado();
        delay(50);
      }
      noInterrupts();
      leftHallTotal = 0;
      rightHallTotal = 0;
      leftHallCount = 0;
      rightHallCount = 0;
      #ifdef ENABLE_OPTO_ENCODERS
      leftOptoCount = 0;
      rightOptoCount = 0;
      #endif
      interrupts();
      if (side == 'L') {
        #ifdef ENABLE_ADAPTIVE_OPTO_FILTER
        leftOptoFilterUs = optoLeftBootstrapFilterForPwm((uint8_t)test_pwm);
        lastHallPulseTimeLeft = 0;
        hallPulseIntervalLeft = 0;
        #endif
        digitalWrite(DIR_LEFT_MOTOR, HIGH);
        motor_pwm_write(PWM_LEFT_MOTOR, test_pwm);
      } else {
        #ifdef ENABLE_ADAPTIVE_OPTO_FILTER
        rightOptoFilterUs = optoRightBootstrapFilterForPwm((uint8_t)test_pwm);
        lastHallPulseTimeRight = 0;
        hallPulseIntervalRight = 0;
        #endif
        digitalWrite(DIR_RIGHT_MOTOR, LOW);
        motor_pwm_write(PWM_RIGHT_MOTOR, test_pwm);
      }
      delay(1000);
      motor_pwm_write(PWM_LEFT_MOTOR, 0);
      motor_pwm_write(PWM_RIGHT_MOTOR, 0);
      noInterrupts();
      uint32_t q_left = leftHallTotal;
      uint32_t q_right = rightHallTotal;
      #ifdef ENABLE_OPTO_ENCODERS
      uint32_t q_opto_left = leftOptoCount;
      uint32_t q_opto_right = rightOptoCount;
      #endif
      interrupts();
      setStateInhabilitado();
      Serial.print("q OK side="); Serial.print(side);
      Serial.print(" pwm="); Serial.print(test_pwm);
      Serial.print(" L="); Serial.print(q_left);
      Serial.print(" R="); Serial.print(q_right);
      #ifdef ENABLE_OPTO_ENCODERS
      Serial.print(" OL="); Serial.print(q_opto_left);
      Serial.print(" OR="); Serial.println(q_opto_right);
      #else
      Serial.println();
      #endif
      break;
    }
    case 'j': {
      // j <microsegundos> - ajuste temporal del filtro de optoencoders.
      // Solo se cambia con los motores detenidos y queda limitado a un rango seguro.
      #ifdef ENABLE_OPTO_ENCODERS
      int sp = cmd.indexOf(' ');
      if (sp < 0) {
        Serial.print("j OK us="); Serial.println(leftOptoFilterUs);
        break;
      }
      String filter_arg = cmd.substring(sp + 1);
      filter_arg.trim();
      if (filter_arg == "stat") {
        noInterrupts();
        uint32_t lraw = leftOptoRawEdges;
        uint32_t lacc = leftOptoAccepted;
        uint32_t lrej = leftOptoRejected;
        uint32_t rraw = rightOptoRawEdges;
        uint32_t racc = rightOptoAccepted;
        uint32_t rrej = rightOptoRejected;
        uint32_t lfilter_us = leftOptoFilterUs;
        uint32_t rfilter_us = rightOptoFilterUs;
        interrupts();
        Serial.print("j STAT Lus="); Serial.print(lfilter_us);
        Serial.print(" Rus="); Serial.print(rfilter_us);
        Serial.print(" Lraw="); Serial.print(lraw);
        Serial.print(" Lacc="); Serial.print(lacc);
        Serial.print(" Lrej="); Serial.print(lrej);
        Serial.print(" Rraw="); Serial.print(rraw);
        Serial.print(" Racc="); Serial.print(racc);
        Serial.print(" Rrej="); Serial.println(rrej);
        break;
      }
      if (filter_arg == "reset") {
        noInterrupts();
        leftOptoRawEdges = 0;
        rightOptoRawEdges = 0;
        leftOptoAccepted = 0;
        rightOptoAccepted = 0;
        leftOptoRejected = 0;
        rightOptoRejected = 0;
        interrupts();
        Serial.println("j RESET OK");
        break;
      }
      uint32_t requested = (uint32_t)filter_arg.toInt();
      requested = constrain(requested, 100UL, 20000UL);
      setLeftMotor(0, true);
      setRightMotor(0, false);
      noInterrupts();
      leftOptoFilterUs = requested;
      rightOptoFilterUs = requested;
      leftOptoLastPulseUs = 0;
      rightOptoLastPulseUs = 0;
      interrupts();
      Serial.print("j OK us="); Serial.println(requested);
      #else
      Serial.println("j FAIL opto_disabled");
      #endif
      break;
    }
    case 'u': {
      // u: una revolucion cerrada por optoencoder (PPR_OPTO_ENCODERS).
      // Cada motor se detiene individualmente al alcanzar el objetivo.
      #ifdef ENABLE_OPTO_ENCODERS
      uint32_t target = PPR_OPTO_ENCODERS;
      int target_sep = cmd.indexOf(' ');
      if (target_sep > 0) {
        target = constrain(cmd.substring(target_sep + 1).toInt(), 1, 600);
      }
      const uint8_t base_pwm = 40;
      const uint8_t min_pwm = 24;
      const unsigned long timeout_ms = 8000;

      setLeftMotor(0, true);
      setRightMotor(0, false);
      if (currentRobotState != STATE_HABILITADO) {
        setStateHabilitado();
        delay(50);
      }
      noInterrupts();
      leftOptoCount = 0;
      rightOptoCount = 0;
      leftHallTotal = 0;
      rightHallTotal = 0;
      leftHallCount = 0;
      rightHallCount = 0;
      interrupts();

      unsigned long started = millis();
      bool left_done = false;
      bool right_done = false;
      while ((!left_done || !right_done) && millis() - started < timeout_ms) {
        noInterrupts();
        uint32_t ol = leftOptoCount;
        uint32_t oright = rightOptoCount;
        interrupts();
        left_done = ol >= target;
        right_done = oright >= target;

        int32_t error = (int32_t)ol - (int32_t)oright;
        int left_pwm = base_pwm;
        int right_pwm = base_pwm;
        if (error > 1) left_pwm -= min((int32_t)12, error * 2);
        if (error < -1) right_pwm -= min((int32_t)12, -error * 2);
        uint32_t left_remaining = ol < target ? target - ol : 0;
        uint32_t right_remaining = oright < target ? target - oright : 0;
        if (left_remaining <= 8) left_pwm = min(left_pwm, 26);
        else if (left_remaining <= 20) left_pwm = min(left_pwm, 32);
        if (right_remaining <= 8) right_pwm = min(right_pwm, 26);
        else if (right_remaining <= 20) right_pwm = min(right_pwm, 32);
        left_pwm = constrain(left_pwm, min_pwm, base_pwm);
        right_pwm = constrain(right_pwm, min_pwm, base_pwm);

        if (left_done) motor_pwm_write(PWM_LEFT_MOTOR, 0);
        else {
          digitalWrite(DIR_LEFT_MOTOR, HIGH);
          motor_pwm_write(PWM_LEFT_MOTOR, left_pwm);
        }
        if (right_done) motor_pwm_write(PWM_RIGHT_MOTOR, 0);
        else {
          digitalWrite(DIR_RIGHT_MOTOR, LOW);
          motor_pwm_write(PWM_RIGHT_MOTOR, right_pwm);
        }
        delay(2);
      }

      motor_pwm_write(PWM_LEFT_MOTOR, 0);
      motor_pwm_write(PWM_RIGHT_MOTOR, 0);
      noInterrupts();
      uint32_t final_ol = leftOptoCount;
      uint32_t final_or = rightOptoCount;
      uint32_t final_hl = leftHallTotal;
      uint32_t final_hr = rightHallTotal;
      interrupts();
      unsigned long elapsed = millis() - started;
      setStateInhabilitado();

      Serial.print("u ");
      Serial.print((final_ol >= target && final_or >= target) ? "OK" : "TIMEOUT");
      Serial.print(" target="); Serial.print(target);
      Serial.print(" OL="); Serial.print(final_ol);
      Serial.print(" OR="); Serial.print(final_or);
      Serial.print(" HL="); Serial.print(final_hl);
      Serial.print(" HR="); Serial.print(final_hr);
      Serial.print(" errL="); Serial.print((int32_t)final_ol - (int32_t)target);
      Serial.print(" errR="); Serial.print((int32_t)final_or - (int32_t)target);
      Serial.print(" revL="); Serial.print((float)final_ol / PPR_OPTO_ENCODERS, 3);
      Serial.print(" revR="); Serial.print((float)final_or / PPR_OPTO_ENCODERS, 3);
      Serial.print(" ms="); Serial.println(elapsed);
      #else
      Serial.println("u FAIL opto_disabled");
      #endif
      break;
    }
    case 'd': {
      // d  — Diagnóstico Hall: imprime el estado raw del pin Hall derecho cada 50ms
      // durante 3s mientras el motor corre a PWM=60. Detecta si el sensor da señal.
      int diagPwm = 60;
      if (currentRobotState != STATE_HABILITADO) { setStateHabilitado(); delay(50); }
      noInterrupts();
      leftHallTotal = 0; rightHallTotal = 0;
      leftHallCount = 0; rightHallCount = 0;
      interrupts();
      setLeftMotor(diagPwm, true);
      setRightMotor(diagPwm, false);
      Serial.println("d DIAG_START pwm=60 3000ms");
      unsigned long t0 = millis();
      uint8_t last_r = 2, last_l = 2;  // estado previo (2=desconocido)
      while (millis() - t0 < 3000) {
        uint8_t r = digitalRead(HALL_RIGHT_MOTOR);
        uint8_t l = digitalRead(HALL_LEFT_MOTOR);
        if (r != last_r || l != last_l) {
          Serial.print("d PIN L="); Serial.print(l);
          Serial.print(" R="); Serial.print(r);
          Serial.print(" t="); Serial.println(millis() - t0);
          last_r = r; last_l = l;
        }
        delay(1);
      }
      setLeftMotor(0, true); setRightMotor(0, false);
      noInterrupts();
      uint32_t lt = leftHallTotal; uint32_t rt = rightHallTotal;
      interrupts();
      Serial.print("d DIAG_END L_pulses="); Serial.print(lt);
      Serial.print(" R_pulses="); Serial.println(rt);
      break;
    }
    case 'c':
      pid_calibrate();
      break;
      
    case 'e':
      ros2_processEncoderRequest();
      break;

    case 'o':
      ros2_processOptoRequest();
      break;
      
    case 'r':
      ros2_processEncoderReset();
      break;
      
    case 's':
      ros2_processStatusRequest();
      break;

    case 'z': {
      // z — diagnóstico de todos los pines relevantes (estado lógico actual)
      Serial.println("z PIN_STATUS_START");
      // --- Motor Izquierdo ---
      Serial.print("z L PWM_pin=46 DIR_pin=52 BRAKE_pin=50 STOP_pin=48");
      Serial.print(" | DIR=");    Serial.print(digitalRead(DIR_LEFT_MOTOR));
      Serial.print(" BRAKE=");   Serial.print(digitalRead(BRAKE_LEFT_MOTOR));
      Serial.print(" BRAKE_mode="); Serial.print((DDRB >> 3) & 1 ? "OUTPUT" : "INPUT");  // pin50→PB3
      Serial.print(" STOP=");    Serial.print(digitalRead(STOP_LEFT_MOTOR));
      Serial.print(" STOP_mode=");  Serial.print((DDRL >> 1) & 1 ? "OUTPUT" : "INPUT"); // pin48→PL1
      Serial.print(" enabled="); Serial.print(leftMotor.enabled);
      Serial.print(" braked=");  Serial.print(leftMotor.braked);
      Serial.print(" pwm=");     Serial.println(leftMotor.pwm);
      // --- Motor Derecho ---
      Serial.print("z R PWM_pin=44 DIR_pin=30 BRAKE_pin=28 STOP_pin=26");
      Serial.print(" | DIR=");    Serial.print(digitalRead(DIR_RIGHT_MOTOR));
      Serial.print(" BRAKE=");   Serial.print(digitalRead(BRAKE_RIGHT_MOTOR));
      Serial.print(" BRAKE_mode="); Serial.print((DDRH >> 5) & 1 ? "OUTPUT" : "INPUT"); // pin28→PH5? check
      Serial.print(" STOP=");    Serial.print(digitalRead(STOP_RIGHT_MOTOR));
      Serial.print(" STOP_mode=");  Serial.print((DDRA >> 4) & 1 ? "OUTPUT" : "INPUT"); // pin26→PA4
      Serial.print(" enabled="); Serial.print(rightMotor.enabled);
      Serial.print(" braked=");  Serial.print(rightMotor.braked);
      Serial.print(" pwm=");     Serial.println(rightMotor.pwm);
      // --- Hall Sensors ---
      Serial.print("z HALL L_pin=19 R_pin=18");
      Serial.print(" | L="); Serial.print(digitalRead(HALL_LEFT_MOTOR));
      Serial.print(" R=");   Serial.print(digitalRead(HALL_RIGHT_MOTOR));
      Serial.print(" L_total="); Serial.print(leftHallTotal);
      Serial.print(" R_total="); Serial.print(rightHallTotal);
      Serial.print(" L_rpm=");   Serial.print(currentSpeedLeftHall, 1);
      Serial.print(" R_rpm=");   Serial.println(currentSpeedRightHall, 1);
      // --- Robot State ---
      Serial.print("z STATE=");
      Serial.print(currentRobotState);
      Serial.print(" ros2_connected="); Serial.print(ros2_connected);
      Serial.print(" Kp_v="); Serial.print(Kp_v, 4);
      Serial.print(" Ki_v="); Serial.println(Ki_v, 4);
      Serial.println("z PIN_STATUS_END");
      break;
    }
      
    case 'x': {
      // x — sweep PWM solo motor DERECHO (sin izquierdo): descarta estáctica vs firmware
      // Barrido: 20,40,60,80,100,150,200  cada nivel 1.5s  (analogWrite directo, sin guards)
      // Imprime OCR5A/TCCR5A y pulsos Hall por nivel.
      Serial.println("x SWEEP_RIGHT_START");

      // --- Force enable SOLO motor derecho ---
      pinMode(STOP_RIGHT_MOTOR,  INPUT);
      pinMode(BRAKE_RIGHT_MOTOR, INPUT);
      rightMotor.enabled = true;  rightMotor.braked = false;
      currentRobotState  = STATE_HABILITADO;
      // Izquierdo APAGADO para no competir por corriente
      analogWrite(PWM_LEFT_MOTOR, 0);
      delay(100);

      // --- DIR derecho = FWD ---
      digitalWrite(DIR_RIGHT_MOTOR, LOW);

      const uint8_t levels[] = {20, 40, 60, 80, 100, 150, 200};
      const uint8_t nLevels  = sizeof(levels);

      for (uint8_t i = 0; i < nLevels; i++) {
        uint8_t pwm = levels[i];
        noInterrupts();
        uint32_t r0 = rightHallTotal;
        interrupts();

        analogWrite(PWM_RIGHT_MOTOR, pwm);
        Serial.print("x PWM="); Serial.print(pwm);
        Serial.print(" OCR5A="); Serial.print(OCR5A);
        Serial.print(" TCCR5A=0x"); Serial.print(TCCR5A, HEX);
        Serial.print(" ...");

        // Esperar 1500ms contando transiciones Hall
        uint8_t xprev = 2;
        uint16_t transitions = 0;
        unsigned long xt0 = millis();
        while (millis() - xt0 < 1500) {
          uint8_t xr = digitalRead(HALL_RIGHT_MOTOR);
          if (xr != xprev && xprev != 2) transitions++;
          xprev = xr;
          delay(1);
        }

        analogWrite(PWM_RIGHT_MOTOR, 0);
        noInterrupts();
        uint32_t r1 = rightHallTotal;
        interrupts();

        Serial.print(" pulses="); Serial.print(r1 - r0);
        Serial.print(" trans=");  Serial.println(transitions);
        delay(300);  // pausa entre niveles
      }

      Serial.println("x SWEEP_RIGHT_END");
      // Reinicializar Timer5 para PWM normal
      initTimer5PWM();
      break;
    }

    case 'X': {
      // X — sweep PWM solo motor IZQUIERDO (mirror de 'x' para el derecho)
      Serial.println("X SWEEP_LEFT_START");

      pinMode(STOP_LEFT_MOTOR,  INPUT);
      pinMode(BRAKE_LEFT_MOTOR, INPUT);
      leftMotor.enabled = true;  leftMotor.braked = false;
      currentRobotState = STATE_HABILITADO;
      analogWrite(PWM_RIGHT_MOTOR, 0);
      delay(100);

      // DIR izquierdo FWD = HIGH (ver Motor_Control.h: DIR_LEFT setHIGH=FWD)
      digitalWrite(DIR_LEFT_MOTOR, HIGH);

      const uint8_t levelsL[] = {20, 40, 60, 80, 100, 150, 200};
      const uint8_t nLevelsL  = sizeof(levelsL);

      for (uint8_t i = 0; i < nLevelsL; i++) {
        uint8_t pwm = levelsL[i];
        noInterrupts();
        uint32_t l0 = leftHallTotal;
        interrupts();

        analogWrite(PWM_LEFT_MOTOR, pwm);
        Serial.print("X PWM="); Serial.print(pwm);
        Serial.print(" OCR5C="); Serial.print(OCR5C);
        Serial.print(" TCCR5A=0x"); Serial.print(TCCR5A, HEX);
        Serial.print(" ...");

        uint8_t xprev = 2;
        uint16_t transitions = 0;
        unsigned long xt0 = millis();
        while (millis() - xt0 < 1500) {
          uint8_t xl = digitalRead(HALL_LEFT_MOTOR);
          if (xl != xprev && xprev != 2) transitions++;
          xprev = xl;
          delay(1);
        }

        analogWrite(PWM_LEFT_MOTOR, 0);
        noInterrupts();
        uint32_t l1 = leftHallTotal;
        interrupts();

        Serial.print(" pulses="); Serial.print(l1 - l0);
        Serial.print(" trans=");  Serial.println(transitions);
        delay(300);
      }

      Serial.println("X SWEEP_LEFT_END");
      initTimer5PWM();
      break;
    }

    case 'f': {
      // f — auto-calibracion FF: barre ambos motores (secuencial),
      // mide pulsos Hall a PWM=60 y 80, calcula FF_*_GAIN y FF_*_BWD,
      // los aplica en RAM y los imprime para copiar a pid_control.h.
      // Uso: enviar 'f' con el robot en marcha libre o suelo.
      Serial.println("f AUTOCAL_FF_START");
      const float PPR_F  = (float)PPR_HALL_SENSORS;
      const float DIAM_F = (float)WHEEL_DIAMETER_M;
      const float MS_PER_WINDOW = 1500.0f;
      // Funcion local: mide RPM a un PWM dado en la rueda indicada
      // (0=izq FWD, 1=der FWD, 2=izq BWD, 3=der BWD)
      auto measureRPM = [&](uint8_t side, uint8_t pwm) -> float {
        noInterrupts();
        uint32_t t0 = (side < 2) ? leftHallTotal : rightHallTotal;
        interrupts();
        if (side == 0) { // izq FWD
          digitalWrite(DIR_LEFT_MOTOR, HIGH);
          analogWrite(PWM_LEFT_MOTOR, pwm);
          analogWrite(PWM_RIGHT_MOTOR, 0);
        } else if (side == 1) { // der FWD
          digitalWrite(DIR_RIGHT_MOTOR, LOW);
          analogWrite(PWM_RIGHT_MOTOR, pwm);
          analogWrite(PWM_LEFT_MOTOR, 0);
        } else if (side == 2) { // izq BWD
          digitalWrite(DIR_LEFT_MOTOR, LOW);
          analogWrite(PWM_LEFT_MOTOR, pwm);
          analogWrite(PWM_RIGHT_MOTOR, 0);
        } else { // der BWD
          digitalWrite(DIR_RIGHT_MOTOR, HIGH);
          analogWrite(PWM_RIGHT_MOTOR, pwm);
          analogWrite(PWM_LEFT_MOTOR, 0);
        }
        delay((unsigned long)MS_PER_WINDOW);
        analogWrite(PWM_LEFT_MOTOR, 0);
        analogWrite(PWM_RIGHT_MOTOR, 0);
        noInterrupts();
        uint32_t t1 = (side < 2) ? leftHallTotal : rightHallTotal;
        interrupts();
        uint32_t pulses = t1 - t0;
        float rpm = (pulses / PPR_F) * (60000.0f / MS_PER_WINDOW);
        return rpm;
      };

      if (currentRobotState != STATE_HABILITADO) { setStateHabilitado(); delay(50); }
      // Disable STOP/BRAKE for direct analogWrite
      pinMode(STOP_LEFT_MOTOR,  INPUT); pinMode(STOP_RIGHT_MOTOR,  INPUT);
      pinMode(BRAKE_LEFT_MOTOR, INPUT); pinMode(BRAKE_RIGHT_MOTOR, INPUT);
      leftMotor.enabled  = rightMotor.enabled  = true;
      leftMotor.braked   = rightMotor.braked   = false;
      delay(200);

      const uint8_t CAL_PWM = 60;
      float rpm_lf = measureRPM(0, CAL_PWM); delay(500);
      float rpm_rf = measureRPM(1, CAL_PWM); delay(500);
      float rpm_lb = measureRPM(2, CAL_PWM); delay(500);
      float rpm_rb = measureRPM(3, CAL_PWM); delay(500);

      // v = rpm * pi * D / 60
      const float K = PI * DIAM_F / 60.0f;
      float v_lf = rpm_lf * K;
      float v_rf = rpm_rf * K;
      float v_lb = rpm_lb * K;
      float v_rb = rpm_rb * K;

      // FF = PWM / v  (con proteccion division por cero)
      if (v_lf > 0.01f) FF_LEFT_GAIN  = (float)CAL_PWM / v_lf;
      if (v_rf > 0.01f) FF_RIGHT_GAIN = (float)CAL_PWM / v_rf;
      if (v_lb > 0.01f) FF_LEFT_BWD   = (float)CAL_PWM / v_lb;
      if (v_rb > 0.01f) FF_RIGHT_BWD  = (float)CAL_PWM / v_rb;

      Serial.println("f AUTOCAL_FF_END");
      Serial.print("f PWM="); Serial.print(CAL_PWM);
      Serial.print(" LF_rpm="); Serial.print(rpm_lf,1);
      Serial.print(" RF_rpm="); Serial.print(rpm_rf,1);
      Serial.print(" LB_rpm="); Serial.print(rpm_lb,1);
      Serial.print(" RB_rpm="); Serial.println(rpm_rb,1);
      Serial.print("f FF_LEFT_GAIN=");  Serial.print(FF_LEFT_GAIN,2);
      Serial.print(" FF_RIGHT_GAIN="); Serial.print(FF_RIGHT_GAIN,2);
      Serial.print(" FF_LEFT_BWD=");   Serial.print(FF_LEFT_BWD,2);
      Serial.print(" FF_RIGHT_BWD=");  Serial.println(FF_RIGHT_BWD,2);
      Serial.println("f (copia estos valores a pid_control.h y reflashea para persistir)");
      initTimer5PWM();
      break;
    }

    case 'y': {
      // y — Replica EXACTAMENTE TEST_MINIMAL_MOTORES desde firmware principal.
      // Ciclo BRAKE + STOP + barrido PWM 10→80 (ambos motores), 1s por nivel.
      // Diagnostica si el motor derecho responde con la secuencia de referencia.
      Serial.println("y MINIMAL_TEST_START");

      // Reset driver: mismo ciclo que TEST_MINIMAL_MOTORES setup()
      pinMode(STOP_LEFT_MOTOR,  OUTPUT); digitalWrite(STOP_LEFT_MOTOR,  LOW);
      pinMode(STOP_RIGHT_MOTOR, OUTPUT); digitalWrite(STOP_RIGHT_MOTOR, LOW);
      pinMode(BRAKE_LEFT_MOTOR,  OUTPUT); digitalWrite(BRAKE_LEFT_MOTOR,  HIGH);
      pinMode(BRAKE_RIGHT_MOTOR, OUTPUT); digitalWrite(BRAKE_RIGHT_MOTOR, HIGH);
      delay(500);
      pinMode(BRAKE_LEFT_MOTOR,  INPUT);
      pinMode(BRAKE_RIGHT_MOTOR, INPUT);
      pinMode(STOP_LEFT_MOTOR,  INPUT);
      pinMode(STOP_RIGHT_MOTOR, INPUT);
      leftMotor.enabled = true;  leftMotor.braked  = false;
      rightMotor.enabled = true; rightMotor.braked = false;
      currentRobotState = STATE_HABILITADO;

      // Barrido idéntico: 10→80 paso 5, 1s por nivel
      const uint8_t yLevels[] = {10,15,20,25,30,35,40,45,50,55,60,65,70,75,80};
      const uint8_t yN = sizeof(yLevels);
      for (uint8_t i = 0; i < yN; i++) {
        uint8_t p = yLevels[i];
        noInterrupts();
        uint32_t l0 = leftHallTotal; uint32_t r0 = rightHallTotal;
        interrupts();

        digitalWrite(DIR_LEFT_MOTOR,  HIGH);  analogWrite(PWM_LEFT_MOTOR,  p);
        digitalWrite(DIR_RIGHT_MOTOR, LOW);   analogWrite(PWM_RIGHT_MOTOR, p);

        delay(1000);

        noInterrupts();
        uint32_t lp = leftHallTotal - l0; uint32_t rp = rightHallTotal - r0;
        interrupts();

        Serial.print("y PWM="); Serial.print(p);
        Serial.print(" L="); Serial.print(lp);
        Serial.print(" R="); Serial.println(rp);
        delay(300);
      }

      // Restaurar estado inhabilitado
      setStateInhabilitado();
      Serial.println("y MINIMAL_TEST_END");
      break;
    }

    default:
      // Comando 'hb' para modo hoverboard (multi-char: empieza con 'h')
      #if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)
      if (command == 'h' && cmd.length() >= 2 && cmd.charAt(1) == 'b') {
        String args = cmd.length() > 3 ? cmd.substring(3) : "";
        hoverboard_processCommand(args);
        break;
      }
      #endif
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
    else if ((first_char == 'e' || first_char == 'o' || first_char == 'r' || first_char == 's' || first_char == 'c') && cmd.length() == 1) {
      ros2_processCommand(cmd);
      return true;
    }
    // k <kp>  : set Kp_v at runtime (calibracion)
    else if (first_char == 'k' && (cmd.length() == 1 || cmd.charAt(1) == ' ')) {
      ros2_processCommand(cmd);
      return true;
    }
    // p <pwm> : raw open-loop PWM ambos motores FWD (medicion planta)
    else if (first_char == 'p' && (cmd.length() == 1 || cmd.charAt(1) == ' ')) {
      ros2_processCommand(cmd);
      return true;
    }
    // q <L|R> <pwm> : prueba individual limitada para calibrar umbral
    else if (first_char == 'q' && cmd.length() >= 5 && cmd.charAt(1) == ' ') {
      ros2_processCommand(cmd);
      return true;
    }
    // j <us> : ajusta el filtro temporal de optoencoders para calibracion.
    else if (first_char == 'j' && (cmd.length() == 1 || cmd.charAt(1) == ' ')) {
      ros2_processCommand(cmd);
      return true;
    }
    else if (first_char == 'n' && (cmd.length() == 1 || cmd.charAt(1) == ' ')) {
      ros2_processCommand(cmd);
      return true;
    }
    // u : una revolucion por motor en lazo cerrado usando optoencoders.
    else if (first_char == 'u' && (cmd.length() == 1 || cmd.charAt(1) == ' ')) {
      ros2_processCommand(cmd);
      return true;
    }
    // d : diagnostico Hall raw (lee pin state mientras motor corre 3s)
    else if (first_char == 'd' && cmd.length() == 1) {
      ros2_processCommand(cmd);
      return true;
    }
    // z : status de todos los pines
    else if (first_char == 'z' && cmd.length() == 1) {
      ros2_processCommand(cmd);
      return true;
    }
    // x : bypass total hardware (OCR5A/TCCR5A dump + direct PWM sin guards)
    else if (first_char == 'x' && cmd.length() == 1) {
      ros2_processCommand(cmd);
      return true;
    }
    // y : replica TEST_MINIMAL_MOTORES (ciclo BRAKE + barrido 10→80) desde firmware principal
    else if (first_char == 'y' && cmd.length() == 1) {
      ros2_processCommand(cmd);
      return true;
    }
    // hc [on|off|reset|stat|filter bias alpha deadband]
    #if defined(ENABLE_MPU9250) && defined(ENABLE_MPU_HEADING_CONTROL)
    else if (first_char == 'h' && cmd.length() >= 2 && cmd.charAt(1) == 'c') {
      ros2_processCommand(cmd);
      return true;
    }
    #endif
    // hb [on|off|cal|stat] : modo hoverboard
    #if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)
    else if (first_char == 'h' && cmd.length() >= 2 && cmd.charAt(1) == 'b') {
      ros2_processCommand(cmd);
      return true;
    }
    #endif
  }
  
  return false;
}

//===========================================================================
//==================== UPDATE LOOP ==========================================
//===========================================================================

void ros2_update() {
  #if defined(ENABLE_HALL_SENSORS) && defined(ENABLE_OPTO_ENCODERS)
  encoder_supervisor_update();
  #endif
  // Verificar timeout de comandos
  if (ros2_connected && (millis() - ros2_last_cmd_time > ROS2_CMD_TIMEOUT)) {
    ros2_linear_vel = 0.0;
    ros2_angular_vel = 0.0;
    ros2_connected = false;
    ros2_clear_pwm_demands();
    
    setLeftMotor(0, true);
    setRightMotor(0, false);  // false = DIR electricamente invertido en motor derecho
    // IMPORTANTE: setRightMotor(0, false) guarda rightMotor.direction=false.
    // Si se usara (0, true), en el siguiente comando v>0 el stop-before-reverse
    // dispararía motor_pwm_write(44,0) en cada ciclo hasta que la dirección
    // se sincronice, causando una pausa visible en el arranque del motor derecho.
    pid_reset_velocity();
    pid_per_wheel_reset();
    
    ros2_status = ROS2_STATUS_WARNING;
  }
  if (ros2_connected) {
    ros2_apply_pwm_demands();
  }
}

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
