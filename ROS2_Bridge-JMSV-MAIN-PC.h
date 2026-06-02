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

bool ros2_connected = false;
unsigned long ros2_last_cmd_time = 0;

float ros2_linear_vel = 0.0;   // m/s
float ros2_angular_vel = 0.0;  // rad/s

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
      
      // Dirección: usar velocidad individual de cada rueda (cinemática diferencial)
      // v_left = linear + angular*(wb/2)  → positivo = adelante
      // v_right = linear - angular*(wb/2) → positivo = adelante
      // Motor derecho: dirForwardRight=LOW → direction=false=LOW=adelante → !dir_right
      const float wb_half = 0.41f;  // wheelbase/2 = 0.82m / 2
      bool dir_left  = (linear + angular * wb_half) >= 0.0f;
      bool dir_right = (linear - angular * wb_half) >= 0.0f;
      
      setLeftMotor(pwm_left,  dir_left);
      setRightMotor(pwm_right, !dir_right);  // DER: LOW=adelante, HIGH=atras
      
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
    Serial.print("e ");
    Serial.print(leftHallCount);
    Serial.print(" ");
    Serial.println(rightHallCount);
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

void ros2_processCommand(String cmd) {
  cmd.trim();
  
  if (cmd.length() == 0) return;
  
  char command = cmd.charAt(0);
  
  switch (command) {
    case 'v':
      ros2_processCmdVel(cmd);
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
    else if ((first_char == 'e' || first_char == 'r' || first_char == 's' || first_char == 'c') && cmd.length() == 1) {
      ros2_processCommand(cmd);
      return true;
    }
  }
  
  return false;
}

//===========================================================================
//==================== UPDATE LOOP ==========================================
//===========================================================================

void ros2_update() {
  // Verificar timeout de comandos
  if (ros2_connected && (millis() - ros2_last_cmd_time > ROS2_CMD_TIMEOUT)) {
    ros2_linear_vel = 0.0;
    ros2_angular_vel = 0.0;
    ros2_connected = false;
    
    setLeftMotor(0, true);
    setRightMotor(0, true);
    
    ros2_status = ROS2_STATUS_WARNING;
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
