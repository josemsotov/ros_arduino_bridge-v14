/**
 * PID_Control.h - Controlador PID para velocidad y posición
 * MOTOR-INTERFACE-V-13
 *
 * Permite control en lazo cerrado de velocidad lineal, angular y posición
 * Incluye método de calibración interactivo
 */

#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <Arduino.h>

// =================== PARÁMETROS PID (ajustar/calibrar) ===================
float Kp_v = 1.0, Ki_v = 0.0, Kd_v = 0.0; // Velocidad lineal
float Kp_w = 1.0, Ki_w = 0.0, Kd_w = 0.0; // Velocidad angular
// Kp_pos ajustado para generar PWM=30 en pruebas: dist_error(1.0m) * Kp_pos(0.30) * 100 = 30 PWM
float Kp_pos = 0.30, Ki_pos = 0.0, Kd_pos = 0.0; // AJUSTADO PARA PRUEBAS: PWM~30

// =================== VARIABLES INTERNAS ===================
float pid_v_integral = 0, pid_v_prev_error = 0;
float pid_w_integral = 0, pid_w_prev_error = 0;
float pid_pos_integral = 0, pid_pos_prev_error = 0;

unsigned long pid_last_time = 0;

// =================== FUNCIONES PID ===================

float pid_compute(float setpoint, float measured, float &integral, float &prev_error, float Kp, float Ki, float Kd, float dt) {
  float error = setpoint - measured;
  integral += error * dt;
  float derivative = (error - prev_error) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  prev_error = error;
  return output;
}

// =================== CONTROL DE VELOCIDAD ===================

void pid_control_velocity(float v_ref, float w_ref, float v_meas, float w_meas, int &pwm_left, int &pwm_right) {
  unsigned long now = millis();
  float dt = (now - pid_last_time) / 1000.0;
  if (dt <= 0) dt = 0.01;
  pid_last_time = now;

  // PID velocidad lineal
  float v_out = pid_compute(v_ref, v_meas, pid_v_integral, pid_v_prev_error, Kp_v, Ki_v, Kd_v, dt);
  // PID velocidad angular
  float w_out = pid_compute(w_ref, w_meas, pid_w_integral, pid_w_prev_error, Kp_w, Ki_w, Kd_w, dt);

  // ⚠️ CINEMÁTICA DIFERENCIAL - NO MODIFICAR SIGNOS ⚠️
  // VERIFICADO: 2025-10-12 - Funciona correctamente con Motor_Control.h
  // v_left = v + w*L/2  (signo +)
  // v_right = v - w*L/2 (signo -)
  const float wheelbase_m = 0.82; // metros
  const float wheel_radius_m = 0.10; // metros

  float v_left = v_out + (w_out * wheelbase_m / 2.0);   // ✅ CORRECTO: + para izquierda
  float v_right = v_out - (w_out * wheelbase_m / 2.0);  // ✅ CORRECTO: - para derecha

  // Convertir a PWM (ajustar factor)
  pwm_left = constrain((int)(abs(v_left) * 100), 0, 255);
  pwm_right = constrain((int)(abs(v_right) * 100), 0, 255);
}

// =================== CONTROL DE POSICIÓN ===================

void pid_control_position(float x_ref, float y_ref, float theta_ref, float x, float y, float theta, int &pwm_left, int &pwm_right) {
  unsigned long now = millis();
  float dt = (now - pid_last_time) / 1000.0;
  if (dt <= 0) dt = 0.01;
  pid_last_time = now;

  // Error de posición
  float dx = x_ref - x;
  float dy = y_ref - y;
  float dtheta = theta_ref - theta;

  // Control PID sobre distancia y ángulo
  float dist_error = sqrt(dx*dx + dy*dy);
  float angle_error = dtheta;

  // PID: setpoint es la distancia objetivo, measured es 0 (posición actual)
  // Esto genera un output positivo proporcional a la distancia
  float v_out = pid_compute(dist_error, 0, pid_pos_integral, pid_pos_prev_error, Kp_pos, Ki_pos, Kd_pos, dt);
  float w_out = pid_compute(angle_error, 0, pid_w_integral, pid_w_prev_error, Kp_w, Ki_w, Kd_w, dt);

  // ⚠️ CINEMÁTICA DIFERENCIAL - NO MODIFICAR SIGNOS ⚠️
  // VERIFICADO: 2025-10-12 - Funciona correctamente con Motor_Control.h
  // Para movimiento recto: v_left = v_right = v
  // Para giro: motor exterior más rápido, interior más lento
  // v_left = v + w*L/2  (signo +)
  // v_right = v - w*L/2 (signo -)
  const float wheelbase_m = 0.82;
  const float wheel_radius_m = 0.10;

  float v_left = v_out + (w_out * wheelbase_m / 2.0);   // ✅ CORRECTO: + para izquierda
  float v_right = v_out - (w_out * wheelbase_m / 2.0);  // ✅ CORRECTO: - para derecha

  // Convertir a PWM - el output ya tiene el signo correcto
  pwm_left = constrain((int)(abs(v_left) * 100), 0, 255);
  pwm_right = constrain((int)(abs(v_right) * 100), 0, 255);
}

// =================== CALIBRACIÓN INTERACTIVA ===================

void pid_calibrate() {
  Serial.println("\n=== CALIBRACIÓN PID ===");
  Serial.println("Envia: KP <valor>, KI <valor>, KD <valor>");
  Serial.println("Ejemplo: KP 2.0");
  Serial.println("Escribe 'EXIT' para salir");
  String input;
  while (true) {
    if (Serial.available()) {
      input = Serial.readStringUntil('\n');
      input.trim();
      if (input.equalsIgnoreCase("EXIT")) break;
      if (input.startsWith("KP ")) Kp_v = input.substring(3).toFloat();
      if (input.startsWith("KI ")) Ki_v = input.substring(3).toFloat();
      if (input.startsWith("KD ")) Kd_v = input.substring(3).toFloat();
      Serial.print("KP="); Serial.print(Kp_v);
      Serial.print(" KI="); Serial.print(Ki_v);
      Serial.print(" KD="); Serial.println(Kd_v);
    }
  }
  Serial.println("Fin calibración PID\n");
}

// =================== CALIBRACIÓN AUTOMÁTICA GENERAL ===================

void auto_calibrate_pid_velocity(float setpoint, int steps = 5) {
  Serial.println("=== AUTO CALIBRACIÓN PID VELOCIDAD ===");
  float best_kp = 0, best_ki = 0, best_kd = 0;
  float min_error = 1e6;
  for (float kp = 0.5; kp <= 2.5; kp += 0.5) {
    for (float ki = 0.0; ki <= 0.5; ki += 0.25) {
      for (float kd = 0.0; kd <= 0.5; kd += 0.25) {
        Kp_v = kp; Ki_v = ki; Kd_v = kd;
        float total_error = 0;
        for (int i = 0; i < steps; i++) {
          float v_meas = 0.0; // Debe leerse de sensores reales
          int pwm_left, pwm_right;
          pid_control_velocity(setpoint, 0, v_meas, 0, pwm_left, pwm_right);
          setLeftMotor(pwm_left, true);
          setRightMotor(pwm_right, true);
          delay(500);
          // v_meas = ... (leer velocidad real)
          total_error += abs(setpoint - v_meas);
        }
        float avg_error = total_error / steps;
        Serial.print("KP="); Serial.print(kp);
        Serial.print(" KI="); Serial.print(ki);
        Serial.print(" KD="); Serial.print(kd);
        Serial.print(" Error="); Serial.println(avg_error, 4);
        if (avg_error < min_error) {
          min_error = avg_error;
          best_kp = kp; best_ki = ki; best_kd = kd;
        }
      }
    }
  }
  Serial.println("=== MEJOR PID VELOCIDAD ===");
  Serial.print("KP="); Serial.print(best_kp);
  Serial.print(" KI="); Serial.print(best_ki);
  Serial.print(" KD="); Serial.println(best_kd);
  Kp_v = best_kp; Ki_v = best_ki; Kd_v = best_kd;
}

void auto_calibrate_pid_position(float x_ref, float y_ref, float theta_ref, int steps = 5) {
  Serial.println("=== AUTO CALIBRACIÓN PID POSICIÓN ===");
  float best_kp = 0, best_ki = 0, best_kd = 0;
  float min_error = 1e6;
  for (float kp = 0.5; kp <= 2.5; kp += 0.5) {
    for (float ki = 0.0; ki <= 0.5; ki += 0.25) {
      for (float kd = 0.0; kd <= 0.5; kd += 0.25) {
        Kp_pos = kp; Ki_pos = ki; Kd_pos = kd;
        float total_error = 0;
        for (int i = 0; i < steps; i++) {
          float x = 0, y = 0, theta = 0; // Debe leerse de odometría real
          int pwm_left, pwm_right;
          pid_control_position(x_ref, y_ref, theta_ref, x, y, theta, pwm_left, pwm_right);
          setLeftMotor(pwm_left, true);
          setRightMotor(pwm_right, true);
          delay(500);
          // x, y, theta = ... (leer odometría real)
          float dist_error = sqrt((x_ref-x)*(x_ref-x)+(y_ref-y)*(y_ref-y));
          total_error += dist_error;
        }
        float avg_error = total_error / steps;
        Serial.print("KP="); Serial.print(kp);
        Serial.print(" KI="); Serial.print(ki);
        Serial.print(" KD="); Serial.print(kd);
        Serial.print(" Error="); Serial.println(avg_error, 4);
        if (avg_error < min_error) {
          min_error = avg_error;
          best_kp = kp; best_ki = ki; best_kd = kd;
        }
      }
    }
  }
  Serial.println("=== MEJOR PID POSICIÓN ===");
  Serial.print("KP="); Serial.print(best_kp);
  Serial.print(" KI="); Serial.print(best_ki);
  Serial.print(" KD="); Serial.println(best_kd);
  Kp_pos = best_kp; Ki_pos = best_ki; Kd_pos = best_kd;
}

void auto_calibrate_pid_angular(float theta_ref, int steps = 5) {
  Serial.println("=== AUTO CALIBRACIÓN PID ANGULAR ===");
  float best_kp = 0, best_ki = 0, best_kd = 0;
  float min_error = 1e6;
  for (float kp = 0.5; kp <= 2.5; kp += 0.5) {
    for (float ki = 0.0; ki <= 0.5; ki += 0.25) {
      for (float kd = 0.0; kd <= 0.5; kd += 0.25) {
        Kp_w = kp; Ki_w = ki; Kd_w = kd;
        float total_error = 0;
        for (int i = 0; i < steps; i++) {
          float theta = 0; // Debe leerse de odometría real
          int pwm_left, pwm_right;
          pid_control_velocity(0, theta_ref, 0, theta, pwm_left, pwm_right);
          setLeftMotor(pwm_left, true);
          setRightMotor(pwm_right, true);
          delay(500);
          // theta = ... (leer odometría real)
          total_error += abs(theta_ref - theta);
        }
        float avg_error = total_error / steps;
        Serial.print("KP="); Serial.print(kp);
        Serial.print(" KI="); Serial.print(ki);
        Serial.print(" KD="); Serial.print(kd);
        Serial.print(" Error="); Serial.println(avg_error, 4);
        if (avg_error < min_error) {
          min_error = avg_error;
          best_kp = kp; best_ki = ki; best_kd = kd;
        }
      }
    }
  }
  Serial.println("=== MEJOR PID ANGULAR ===");
  Serial.print("KP="); Serial.print(best_kp);
  Serial.print(" KI="); Serial.print(best_ki);
  Serial.print(" KD="); Serial.println(best_kd);
  Kp_w = best_kp; Ki_w = best_ki; Kd_w = best_kd;
}

void pid_calibrate_general() {
  Serial.println("\n=== CALIBRACIÓN AUTOMÁTICA GENERAL ===");
  Serial.println("Elige tipo: VEL, POS, ANG");
  Serial.println("Ejemplo: VEL");
  String input;
  while (true) {
    if (Serial.available()) {
      input = Serial.readStringUntil('\n');
      input.trim();
      if (input.equalsIgnoreCase("VEL")) {
        auto_calibrate_pid_velocity(0.3, 5);
        break;
      }
      if (input.equalsIgnoreCase("POS")) {
        auto_calibrate_pid_position(1.0, 0.0, 0.0, 5);
        break;
      }
      if (input.equalsIgnoreCase("ANG")) {
        auto_calibrate_pid_angular(1.57, 5);
        break;
      }
      if (input.equalsIgnoreCase("EXIT")) break;
      Serial.println("Opciones: VEL, POS, ANG, EXIT");
    }
  }
  Serial.println("Fin calibración automática\n");
}

#endif