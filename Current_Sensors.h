/**
 * ============================================================================
 * CURRENT_SENSORS.H — Sensores de corriente ACS712
 * Smart Golf Trolley — Monitoreo de corriente en tiempo real
 * ============================================================================
 *
 * HARDWARE:
 *   Motor DERECHO  → ACS712 → A3
 *   Motor IZQUIERDO → ACS712 → A4
 *
 * MODELO ACS712 — SELECCIONAR EN Configuration.h:
 *   ACS712_5A  → sensibilidad 185 mV/A  → corrientes hasta ±5A
 *   ACS712_20A → sensibilidad 100 mV/A  → corrientes hasta ±20A  ← default
 *   ACS712_30A → sensibilidad  66 mV/A  → corrientes hasta ±30A
 *
 * FÓRMULA:
 *   Vout = (ADC / 1023.0) * 5.0          (voltaje en el pin analógico)
 *   I(A) = (Vout - Vzero) / sensitivity   (amperios)
 *   donde Vzero = 2.5V (salida del sensor a 0 A)
 *
 * CALIBRACIÓN DE OFFSET:
 *   Si los motores están parados y la lectura no es 0.0A, ajustar
 *   ACS712_ZERO_OFFSET_L / ACS712_ZERO_OFFSET_R en Configuration.h
 *   hasta leer 0.0A en reposo.
 *
 * FILTRADO:
 *   Filtro EMA (media móvil exponencial) para suavizar el ruido del ADC.
 *   Alpha configurable con ACS712_FILTER_ALPHA en Configuration.h.
 * ============================================================================
 */

#ifndef CURRENT_SENSORS_H
#define CURRENT_SENSORS_H

#ifdef ENABLE_CURRENT_SENSORS

#include <Arduino.h>

//===========================================================================
//==================== VARIABLES GLOBALES ==================================
//===========================================================================

float current_right_A = 0.0f;   // Corriente motor DERECHO (A)
float current_left_A  = 0.0f;   // Corriente motor IZQUIERDO (A)

// Offsets de calibración en runtime (se ajustan con comando 'curr cal')
float _curr_offset_r = 0.0f;
float _curr_offset_l = 0.0f;

// Conversión ADC→voltaje (5V / 1023 pasos)
#define ACS712_ADC_TO_V  (5.0f / 1023.0f)
// Punto cero del sensor: 2.5V → 511.5 ADC ≈ 512
#define ACS712_VZERO     2.5f

//===========================================================================
//==================== INICIALIZACIÓN =====================================
//===========================================================================

void current_sensors_init() {
  pinMode(CURRENT_RIGHT_PIN, INPUT);
  pinMode(CURRENT_LEFT_PIN,  INPUT);

  // Warm-up: descartar las primeras lecturas del ADC
  for (uint8_t i = 0; i < 10; i++) {
    analogRead(CURRENT_RIGHT_PIN);
    analogRead(CURRENT_LEFT_PIN);
  }

  // Calibración automática de offset en reposo (promedio de 64 muestras)
  float sum_r = 0.0f, sum_l = 0.0f;
  for (uint8_t i = 0; i < 64; i++) {
    sum_r += (analogRead(CURRENT_RIGHT_PIN) * ACS712_ADC_TO_V) - ACS712_VZERO;
    sum_l += (analogRead(CURRENT_LEFT_PIN)  * ACS712_ADC_TO_V) - ACS712_VZERO;
    delayMicroseconds(200);
  }
  _curr_offset_r = sum_r / 64.0f;
  _curr_offset_l = sum_l / 64.0f;

  Serial.println(F("[CURR] Sensores ACS712 inicializados"));
  Serial.print(F("[CURR] Offset R="));  Serial.print(_curr_offset_r * 1000, 1);
  Serial.print(F("mV  L="));           Serial.print(_curr_offset_l * 1000, 1);
  Serial.println(F("mV"));
}

//===========================================================================
//==================== LECTURA Y FILTRADO ===================================
//===========================================================================

void current_sensors_update() {
  // Leer ADC y convertir a voltaje relativo al cero del sensor
  float vr = (analogRead(CURRENT_RIGHT_PIN) * ACS712_ADC_TO_V) - ACS712_VZERO - _curr_offset_r;
  float vl = (analogRead(CURRENT_LEFT_PIN)  * ACS712_ADC_TO_V) - ACS712_VZERO - _curr_offset_l;

  // Convertir voltaje a corriente con la sensibilidad configurada
  float ir = vr / ACS712_SENSITIVITY_VA;
  float il = vl / ACS712_SENSITIVITY_VA;

  // Filtro EMA para suavizar ruido
  current_right_A += ACS712_FILTER_ALPHA * (ir - current_right_A);
  current_left_A  += ACS712_FILTER_ALPHA * (il - current_left_A);
}

//===========================================================================
//==================== CALIBRACIÓN MANUAL ==================================
//===========================================================================

void current_sensors_calibrate() {
  // Recalibrar offsets con motores parados
  float sum_r = 0.0f, sum_l = 0.0f;
  for (uint8_t i = 0; i < 64; i++) {
    sum_r += (analogRead(CURRENT_RIGHT_PIN) * ACS712_ADC_TO_V) - ACS712_VZERO;
    sum_l += (analogRead(CURRENT_LEFT_PIN)  * ACS712_ADC_TO_V) - ACS712_VZERO;
    delayMicroseconds(200);
  }
  _curr_offset_r = sum_r / 64.0f;
  _curr_offset_l = sum_l / 64.0f;
  current_right_A = 0.0f;
  current_left_A  = 0.0f;
  Serial.println(F("[CURR] Calibracion completada (motores en reposo)"));
  Serial.print(F("[CURR] Nuevo offset R="));  Serial.print(_curr_offset_r * 1000, 1);
  Serial.print(F("mV  L="));                  Serial.print(_curr_offset_l * 1000, 1);
  Serial.println(F("mV"));
}

#endif // ENABLE_CURRENT_SENSORS
#endif // CURRENT_SENSORS_H
