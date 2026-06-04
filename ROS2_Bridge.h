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
      const float wb2 = WHEEL_BASE_DISTANCE_M / 2.0f;
      float v_left_raw  = linear + angular * wb2;
      float v_right_raw = linear - angular * wb2;
      bool  cmd_dir_left  = (v_left_raw  >= 0.0f);  // direccion comandada izq
      bool  cmd_dir_right = (v_right_raw >= 0.0f);  // direccion comandada der

      // ── CAMBIO DE DIRECCIÓN: stop previo al driver ────────────────────────
      // ZS-X11H (BTS7960) ignora cambio de DIR si el PWM está activo.
      // Al detectar inversión: PWM=0 por 5ms + limpiar Hall stale + limpiar integral.
      static bool prev_cmd_dir_left  = true;
      static bool prev_cmd_dir_right = true;
      if (cmd_dir_left != prev_cmd_dir_left) {
        analogWrite(PWM_LEFT_MOTOR, 0);
        currentSpeedLeftHall = 0.0f;
        integral_left = 0.0f;
        delay(5);
      }
      if (cmd_dir_right != prev_cmd_dir_right) {
        analogWrite(PWM_RIGHT_MOTOR, 0);
        currentSpeedRightHall = 0.0f;
        integral_right = 0.0f;
        delay(5);
      }
      prev_cmd_dir_left  = cmd_dir_left;
      prev_cmd_dir_right = cmd_dir_right;

      // ── PID INDEPENDIENTE POR RUEDA con feedback Hall ─────────────────────
      // El PID acoplado v/w provoca saturación cuando una rueda es mucho más
      // rápida que la otra: la corrección angular w_out se dispara y envía la
      // rueda lenta en reversa. El PID por rueda evita este acoplamiento.
      //
      // e_left  = v_left_ref  - v_left_meas  →  pwm_left  = Kp_v * e * 100
      // e_right = v_right_ref - v_right_meas →  pwm_right = Kp_v * e * 100
      // CLAMP: no cruzar cero (dirección) cuando el setpoint es unidireccional.
      const float rpm_to_ms = PI * WHEEL_DIAMETER_M / 60.0f;
      float v_left_meas  = currentSpeedLeftHall  * rpm_to_ms * (cmd_dir_left  ? 1.0f : -1.0f);
      float v_right_meas = currentSpeedRightHall * rpm_to_ms * (cmd_dir_right ? 1.0f : -1.0f);

      float e_left  = v_left_raw  - v_left_meas;
      float e_right = v_right_raw - v_right_meas;

      // ── INTEGRAL con anti-windup ──────────────────────────────────────────
      unsigned long now_pid = millis();
      if (pid_per_wheel_last_time == 0) pid_per_wheel_last_time = now_pid;
      float dt_pid = (now_pid - pid_per_wheel_last_time) / 1000.0f;
      if (dt_pid <= 0.0f || dt_pid > 2.0f) dt_pid = 0.05f;
      pid_per_wheel_last_time = now_pid;

      const float INTEGRAL_MAX = (float)MAX_PWM_VALUE / (Ki_v > 0.0f ? (Ki_v * 100.0f) : 1.0f);
      integral_left  += e_left  * dt_pid;
      integral_right += e_right * dt_pid;
      // Anti-windup unidireccional: el integral solo acumula en la direccion del setpoint.
      // Evita que un overshoot arrastre el integral negativo y deje el motor en PWM=0.
      if (cmd_dir_left) {
        if (integral_left  < 0.0f) integral_left  = 0.0f;
        if (integral_left  > INTEGRAL_MAX) integral_left  = INTEGRAL_MAX;
      } else {
        if (integral_left  > 0.0f) integral_left  = 0.0f;
        if (integral_left  < -INTEGRAL_MAX) integral_left  = -INTEGRAL_MAX;
      }
      if (cmd_dir_right) {
        if (integral_right < 0.0f) integral_right = 0.0f;
        if (integral_right > INTEGRAL_MAX) integral_right = INTEGRAL_MAX;
      } else {
        if (integral_right > 0.0f) integral_right = 0.0f;
        if (integral_right < -INTEGRAL_MAX) integral_right = -INTEGRAL_MAX;
      }

      // ── PWM = Feedforward + corrección PID ───────────────────────────────
      // FF compensa la asimetría física entre motores (izq ~35-57% más rápido).
      // Kp/Ki sólo corrigen el residuo. v_left_raw lleva el signo de la dirección.
      int pwm_left  = constrain((int)(v_left_raw  * FF_LEFT_GAIN  + (e_left  * Kp_v + integral_left  * Ki_v) * 100.0f), -MAX_PWM_VALUE, MAX_PWM_VALUE);
      int pwm_right = constrain((int)(v_right_raw * FF_RIGHT_GAIN + (e_right * Kp_v + integral_right * Ki_v) * 100.0f), -MAX_PWM_VALUE, MAX_PWM_VALUE);

      // CLAMP ANTI-INVERSIÓN: evitar que la corrección PID invierta la dirección.
      // Con FF dominante esto raramente se activa, pero es una red de seguridad.
      if ( cmd_dir_left  && pwm_left  < 0) pwm_left  = 0;
      if (!cmd_dir_left  && pwm_left  > 0) pwm_left  = 0;
      if ( cmd_dir_right && pwm_right < 0) pwm_right = 0;
      if (!cmd_dir_right && pwm_right > 0) pwm_right = 0;

      // Extraer magnitud y direccion del OUTPUT del PID
      bool dir_left  = (pwm_left  >= 0);
      bool dir_right = (pwm_right >= 0);
      int  abs_left  = abs(pwm_left);
      int  abs_right = abs(pwm_right);

      // Stop-before-reverse: ZS-X11H/BTS7960 ignora cambio de DIR si PWM está activo.
      // Aplicar PWM=0 por 2ms antes de cambiar dirección para que el driver lo registre.
      if (abs_left  > 0 && dir_left   != leftMotor.direction)  { analogWrite(PWM_LEFT_MOTOR,  0); delayMicroseconds(2000); }
      if (abs_right > 0 && !dir_right != rightMotor.direction) { analogWrite(PWM_RIGHT_MOTOR, 0); delayMicroseconds(2000); }

      setLeftMotor(abs_left, dir_left);
      setRightMotor(abs_right, !dir_right);  // Motor derecho: DIR electrica invertida

      // ── Telemetría compacta ────────────────────────────────────────────────
      // Suprimida cuando el balance anti-caída llama esta función a 50 Hz
      #if defined(ENABLE_HOVERBOARD_MODE) && defined(ENABLE_MPU9250)
      if (!bal_suppress_telemetry)
      #endif
      {
        Serial.print("T lin="); Serial.print(linear, 3);
        Serial.print(" ang=");  Serial.print(angular, 3);
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
      Serial.print("z L PWM_pin=44 DIR_pin=30 BRAKE_pin=28 STOP_pin=26");
      Serial.print(" | DIR=");    Serial.print(digitalRead(DIR_LEFT_MOTOR));
      Serial.print(" BRAKE=");   Serial.print(digitalRead(BRAKE_LEFT_MOTOR));
      Serial.print(" BRAKE_mode="); Serial.print((DDRH >> 5) & 1 ? "OUTPUT" : "INPUT");  // pin28→PH5
      Serial.print(" STOP=");    Serial.print(digitalRead(STOP_LEFT_MOTOR));
      Serial.print(" STOP_mode=");  Serial.print((DDRA >> 4) & 1 ? "OUTPUT" : "INPUT"); // pin26→PA4
      Serial.print(" enabled="); Serial.print(leftMotor.enabled);
      Serial.print(" braked=");  Serial.print(leftMotor.braked);
      Serial.print(" pwm=");     Serial.println(leftMotor.pwm);
      // --- Motor Derecho ---
      Serial.print("z R PWM_pin=46 DIR_pin=52 BRAKE_pin=50 STOP_pin=48");
      Serial.print(" | DIR=");    Serial.print(digitalRead(DIR_RIGHT_MOTOR));
      Serial.print(" BRAKE=");   Serial.print(digitalRead(BRAKE_RIGHT_MOTOR));
      Serial.print(" BRAKE_mode="); Serial.print((DDRH >> 3) & 1 ? "OUTPUT" : "INPUT"); // pin50→PH3? check
      Serial.print(" STOP=");    Serial.print(digitalRead(STOP_RIGHT_MOTOR));
      Serial.print(" STOP_mode=");  Serial.print((DDRL >> 1) & 1 ? "OUTPUT" : "INPUT"); // pin48→PL1
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
    else if ((first_char == 'e' || first_char == 'r' || first_char == 's' || first_char == 'c') && cmd.length() == 1) {
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
  // Verificar timeout de comandos
  if (ros2_connected && (millis() - ros2_last_cmd_time > ROS2_CMD_TIMEOUT)) {
    ros2_linear_vel = 0.0;
    ros2_angular_vel = 0.0;
    ros2_connected = false;
    
    setLeftMotor(0, true);
    setRightMotor(0, true);
    pid_reset_velocity();
    pid_per_wheel_reset();
    
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
