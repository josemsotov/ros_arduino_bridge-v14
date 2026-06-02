/**
 * MOTOR-INTERFACE-V-13 MPU9250/6500 MODULE
 * Smart Golf Trolley - Sensor inercial I2C
 *
 * CARACTERÍSTICAS:
 * - Acelerómetro + Giroscopio (MPU6500) o + Magnetómetro (MPU9250)
 * - Filtro DLPF hardware configurable
 * - Filtro EMA software para suavizado adicional
 * - Sample rate: 200 Hz
 * - Integración de ángulo Yaw con compensación de drift
 * - Calibración de offsets en tiempo de boot
 *
 * CONEXIÓN:
 *   VCC → 3.3V
 *   GND → GND
 *   SDA → Pin 20 (I2C Data)
 *   SCL → Pin 21 (I2C Clock)
 *   AD0 → GND  (dirección I2C = 0x68)
 *
 * COMANDOS SERIAL:
 *   MPU_STATUS  → Estado y lectura actual
 *   MPU_CALIBRATE → Recalibrar offsets (robot quieto)
 *   MPU_RESET   → Resetear ángulo integrado
 *   MPU_DEBUG   → Activar/desactivar salida continua
 */

#ifndef MPU9250_H
#define MPU9250_H

#ifdef ENABLE_MPU9250

#include <Wire.h>

//===========================================================================
//========================== CONSTANTES MPU =================================
//===========================================================================

#define MPU_I2C_ADDR        0x68    // AD0 = GND
#define MPU_I2C_ADDR_ALT    0x69    // AD0 = VCC (alternativo)

// Registros MPU6500/9250
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_CONFIG2 0x1D
#define MPU_REG_INT_PIN_CFG  0x37
#define MPU_REG_INT_ENABLE   0x38
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_TEMP_OUT_H   0x41
#define MPU_REG_GYRO_XOUT_H  0x43
#define MPU_REG_USER_CTRL    0x6A
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_PWR_MGMT_2   0x6C
#define MPU_REG_WHO_AM_I     0x75

// WHO_AM_I esperados
#define MPU6500_WHO_AM_I     0x70
#define MPU9250_WHO_AM_I     0x71
#define MPU6050_WHO_AM_I     0x68

// Escalas del giroscopio
#define GYRO_FS_250   0x00    // ±250 °/s  → 131.0 LSB/(°/s)
#define GYRO_FS_500   0x08    // ±500 °/s  → 65.5  LSB/(°/s)
#define GYRO_FS_1000  0x10    // ±1000 °/s → 32.8  LSB/(°/s)
#define GYRO_FS_2000  0x18    // ±2000 °/s → 16.4  LSB/(°/s)

// Escalas del acelerómetro
#define ACCEL_FS_2G   0x00    // ±2g  → 16384 LSB/g
#define ACCEL_FS_4G   0x08    // ±4g  → 8192  LSB/g
#define ACCEL_FS_8G   0x10    // ±8g  → 4096  LSB/g
#define ACCEL_FS_16G  0x18    // ±16g → 2048  LSB/g

// DLPF (Digital Low Pass Filter) - Bandwidth / Delay
// Para GYRO: REG_CONFIG bits[2:0]
#define DLPF_250HZ    0x00    // BW=250Hz, delay=0.97ms   (sin filtro)
#define DLPF_184HZ    0x01    // BW=184Hz, delay=2.9ms
#define DLPF_92HZ     0x02    // BW=92Hz,  delay=3.9ms
#define DLPF_41HZ     0x03    // BW=41Hz,  delay=5.9ms    ← recomendado robot
#define DLPF_20HZ     0x04    // BW=20Hz,  delay=9.9ms
#define DLPF_10HZ     0x05    // BW=10Hz,  delay=17.85ms
#define DLPF_5HZ      0x06    // BW=5Hz,   delay=33.48ms

//===========================================================================
//========================== CONFIGURACIÓN ==================================
//===========================================================================

// Configuración por defecto (ajustable aquí)
#define MPU_GYRO_SCALE      GYRO_FS_500      // ±500 °/s → bueno para robot diferencial
#define MPU_ACCEL_SCALE     ACCEL_FS_4G      // ±4g
#define MPU_DLPF_SETTING    DLPF_41HZ        // 41Hz corta vibraciones del motor
#define MPU_SAMPLE_RATE_DIV 4                // Sample rate = 1000/(1+4) = 200 Hz

// Factor EMA (Exponential Moving Average) para suavizado software
// Rango: 0.0 (sin respuesta) ~ 1.0 (sin filtro)
// 0.1 = muy suave, 0.3 = equilibrado, 0.7 = rápido
#define MPU_EMA_ALPHA       0.15f            // Factor EMA para giroscopio

// Número de muestras para calibración de offsets
#define MPU_CALIBRATION_SAMPLES  200

// Escala real según configuración
#define GYRO_SENSITIVITY    65.5f            // LSB/(°/s) para ±500°/s
#define ACCEL_SENSITIVITY   8192.0f          // LSB/g para ±4g

//===========================================================================
//========================== VARIABLES GLOBALES =============================
//===========================================================================

// Estado del sensor
bool mpu_initialized = false;
bool mpu_found = false;
uint8_t mpu_whoAmI = 0;

// Offsets de calibración
float mpu_gyroOffsetX = 0, mpu_gyroOffsetY = 0, mpu_gyroOffsetZ = 0;
float mpu_accelOffsetX = 0, mpu_accelOffsetY = 0, mpu_accelOffsetZ = 0;

// Lecturas crudas (raw)
int16_t mpu_rawAccelX, mpu_rawAccelY, mpu_rawAccelZ;
int16_t mpu_rawTemp;
int16_t mpu_rawGyroX, mpu_rawGyroY, mpu_rawGyroZ;

// Lecturas en unidades físicas (después de offset y escala)
float mpu_accelX, mpu_accelY, mpu_accelZ;   // g
float mpu_gyroX, mpu_gyroY, mpu_gyroZ;      // °/s
float mpu_tempC;                              // °C

// Lecturas filtradas EMA
float mpu_gyroX_f = 0, mpu_gyroY_f = 0, mpu_gyroZ_f = 0;
float mpu_accelX_f = 0, mpu_accelY_f = 0, mpu_accelZ_f = 0;

// Ángulo Yaw integrado (°) — usado para control de giro y odometría
float mpu_yaw = 0.0f;
float mpu_yaw_reference = 0.0f;   // Referencia para medir giros relativos

// Ángulo Pitch (°) — filtro complementario gyroY + accelX/Z
// Positivo = robot inclinado hacia adelante
float mpu_pitch = 0.0f;
bool  mpu_pitch_initialized = false;  // True tras primera lectura accel

// Tiempo para integración
unsigned long mpu_lastReadTime = 0;
unsigned long mpu_lastUpdateTime = 0;
#define MPU_UPDATE_INTERVAL_MS  5   // 200 Hz = cada 5ms

// Debug continuo
bool mpu_continuousDebug = false;
unsigned long mpu_lastDebugOutput = 0;
#define MPU_DEBUG_INTERVAL_MS   100  // Debug a 10 Hz

//===========================================================================
//========================== FUNCIONES I2C BÁSICAS ==========================
//===========================================================================

void mpu_writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t mpu_readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_I2C_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

void mpu_readRegisters(uint8_t reg, uint8_t count, uint8_t* buf) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_I2C_ADDR, count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

//===========================================================================
//========================== INICIALIZACIÓN =================================
//===========================================================================

/**
 * Verifica que el MPU responde en I2C y lee WHO_AM_I
 */
bool mpu_detect() {
  mpu_whoAmI = mpu_readRegister(MPU_REG_WHO_AM_I);
  if (mpu_whoAmI == MPU6500_WHO_AM_I ||
      mpu_whoAmI == MPU9250_WHO_AM_I ||
      mpu_whoAmI == MPU6050_WHO_AM_I) {
    mpu_found = true;
    return true;
  }
  // Intentar dirección alternativa
  Wire.beginTransmission(MPU_I2C_ADDR_ALT);
  if (Wire.endTransmission() == 0) {
    Serial.println("[MPU] Sensor encontrado en dirección 0x69 (AD0=VCC)");
  }
  return false;
}

/**
 * Inicializar el MPU: despertar, configurar escalas y DLPF
 */
void mpu_initialize() {
  Serial.println("[MPU] Inicializando MPU9250/6500...");

  // 1. Despertar el sensor (quitar sleep bit)
  mpu_writeRegister(MPU_REG_PWR_MGMT_1, 0x80);  // Reset completo
  delay(100);
  mpu_writeRegister(MPU_REG_PWR_MGMT_1, 0x01);  // Clock: PLL con giroscopio X
  delay(50);

  // 2. Verificar comunicación
  if (!mpu_detect()) {
    Serial.print("[MPU] ERROR: No se detectó el sensor. WHO_AM_I=0x");
    Serial.println(mpu_whoAmI, HEX);
    Serial.println("[MPU] Verifica conexión SDA(pin20) SCL(pin21) y alimentación 3.3V");
    mpu_initialized = false;
    return;
  }

  Serial.print("[MPU] Sensor detectado. WHO_AM_I=0x");
  Serial.println(mpu_whoAmI, HEX);
  if (mpu_whoAmI == MPU9250_WHO_AM_I) Serial.println("[MPU] Tipo: MPU9250");
  else if (mpu_whoAmI == MPU6500_WHO_AM_I) Serial.println("[MPU] Tipo: MPU6500");
  else Serial.println("[MPU] Tipo: MPU6050 compatible");

  mpu_initialized = true;
}

/**
 * Configurar DLPF, escalas y sample rate
 */
void mpu_configure() {
  if (!mpu_initialized) return;

  // Sample rate divider → 200 Hz
  mpu_writeRegister(MPU_REG_SMPLRT_DIV, MPU_SAMPLE_RATE_DIV);

  // DLPF hardware para giroscopio
  mpu_writeRegister(MPU_REG_CONFIG, MPU_DLPF_SETTING);

  // Escala del giroscopio
  mpu_writeRegister(MPU_REG_GYRO_CONFIG, MPU_GYRO_SCALE);

  // Escala del acelerómetro
  mpu_writeRegister(MPU_REG_ACCEL_CONFIG, MPU_ACCEL_SCALE);

  // DLPF del acelerómetro (misma banda)
  mpu_writeRegister(MPU_REG_ACCEL_CONFIG2, MPU_DLPF_SETTING);

  // Deshabilitar interrupciones (polling manual)
  mpu_writeRegister(MPU_REG_INT_ENABLE, 0x00);

  mpu_lastReadTime = millis();
  mpu_lastUpdateTime = millis();

  Serial.println("[MPU] Configuración aplicada:");
  Serial.print("  DLPF=41Hz  Gyro=±500°/s  Accel=±4g  SampleRate=200Hz");
  Serial.println();
}

//===========================================================================
//========================== CALIBRACIÓN DE OFFSETS =========================
//===========================================================================

/**
 * Calcula offsets promediando N muestras con el robot estático.
 * LLAMAR SOLO cuando el robot está completamente quieto.
 */
void mpu_calibrateOffsets() {
  if (!mpu_initialized) return;

  Serial.println("[MPU] Calibrando offsets... (robot quieto)");
  Serial.print("  Tomando ");
  Serial.print(MPU_CALIBRATION_SAMPLES);
  Serial.println(" muestras...");

  long sumGX = 0, sumGY = 0, sumGZ = 0;
  long sumAX = 0, sumAY = 0, sumAZ = 0;

  for (int i = 0; i < MPU_CALIBRATION_SAMPLES; i++) {
    uint8_t buf[14];
    mpu_readRegisters(MPU_REG_ACCEL_XOUT_H, 14, buf);

    sumAX += (int16_t)((buf[0]  << 8) | buf[1]);
    sumAY += (int16_t)((buf[2]  << 8) | buf[3]);
    sumAZ += (int16_t)((buf[4]  << 8) | buf[5]);
    sumGX += (int16_t)((buf[8]  << 8) | buf[9]);
    sumGY += (int16_t)((buf[10] << 8) | buf[11]);
    sumGZ += (int16_t)((buf[12] << 8) | buf[13]);

    delay(5);  // 200 Hz
  }

  mpu_gyroOffsetX = (float)sumGX / MPU_CALIBRATION_SAMPLES;
  mpu_gyroOffsetY = (float)sumGY / MPU_CALIBRATION_SAMPLES;
  mpu_gyroOffsetZ = (float)sumGZ / MPU_CALIBRATION_SAMPLES;

  // Para accel: X e Y deben ser 0, Z debe ser 1g (gravedad)
  mpu_accelOffsetX = (float)sumAX / MPU_CALIBRATION_SAMPLES;
  mpu_accelOffsetY = (float)sumAY / MPU_CALIBRATION_SAMPLES;
  mpu_accelOffsetZ = (float)sumAZ / MPU_CALIBRATION_SAMPLES - ACCEL_SENSITIVITY; // restar 1g

  Serial.println("[MPU] Calibración completada:");
  Serial.print("  Gyro offsets (LSB): X="); Serial.print(mpu_gyroOffsetX, 1);
  Serial.print(" Y="); Serial.print(mpu_gyroOffsetY, 1);
  Serial.print(" Z="); Serial.println(mpu_gyroOffsetZ, 1);
  Serial.print("  Accel offsets (LSB): X="); Serial.print(mpu_accelOffsetX, 1);
  Serial.print(" Y="); Serial.print(mpu_accelOffsetY, 1);
  Serial.print(" Z="); Serial.println(mpu_accelOffsetZ, 1);

  // Resetear ángulo integrado
  mpu_yaw = 0.0f;
  mpu_yaw_reference = 0.0f;
}

//===========================================================================
//========================== LECTURA Y FILTRADO =============================
//===========================================================================

/**
 * Lee todos los registros de accel+temp+gyro en una sola transacción I2C (14 bytes)
 * y aplica offsets, escala y filtro EMA.
 * Llamar desde loop() cada MPU_UPDATE_INTERVAL_MS.
 */
void mpu_update() {
  if (!mpu_initialized) return;

  unsigned long now = millis();
  if (now - mpu_lastUpdateTime < MPU_UPDATE_INTERVAL_MS) return;

  float dt = (now - mpu_lastUpdateTime) / 1000.0f;  // segundos
  mpu_lastUpdateTime = now;

  // Leer 14 bytes: ACCEL(6) + TEMP(2) + GYRO(6)
  uint8_t buf[14];
  mpu_readRegisters(MPU_REG_ACCEL_XOUT_H, 14, buf);

  mpu_rawAccelX = (int16_t)((buf[0]  << 8) | buf[1]);
  mpu_rawAccelY = (int16_t)((buf[2]  << 8) | buf[3]);
  mpu_rawAccelZ = (int16_t)((buf[4]  << 8) | buf[5]);
  mpu_rawTemp   = (int16_t)((buf[6]  << 8) | buf[7]);
  mpu_rawGyroX  = (int16_t)((buf[8]  << 8) | buf[9]);
  mpu_rawGyroY  = (int16_t)((buf[10] << 8) | buf[11]);
  mpu_rawGyroZ  = (int16_t)((buf[12] << 8) | buf[13]);

  // Aplicar offsets y convertir a unidades físicas
  mpu_accelX = (mpu_rawAccelX - mpu_accelOffsetX) / ACCEL_SENSITIVITY;
  mpu_accelY = (mpu_rawAccelY - mpu_accelOffsetY) / ACCEL_SENSITIVITY;
  mpu_accelZ = (mpu_rawAccelZ - mpu_accelOffsetZ) / ACCEL_SENSITIVITY;

  mpu_gyroX  = (mpu_rawGyroX - mpu_gyroOffsetX) / GYRO_SENSITIVITY;
  mpu_gyroY  = (mpu_rawGyroY - mpu_gyroOffsetY) / GYRO_SENSITIVITY;
  mpu_gyroZ  = (mpu_rawGyroZ - mpu_gyroOffsetZ) / GYRO_SENSITIVITY;

  mpu_tempC  = mpu_rawTemp / 333.87f + 21.0f;

  // Filtro EMA
  mpu_accelX_f += MPU_EMA_ALPHA * (mpu_accelX - mpu_accelX_f);
  mpu_accelY_f += MPU_EMA_ALPHA * (mpu_accelY - mpu_accelY_f);
  mpu_accelZ_f += MPU_EMA_ALPHA * (mpu_accelZ - mpu_accelZ_f);
  mpu_gyroX_f  += MPU_EMA_ALPHA * (mpu_gyroX  - mpu_gyroX_f);
  mpu_gyroY_f  += MPU_EMA_ALPHA * (mpu_gyroY  - mpu_gyroY_f);
  mpu_gyroZ_f  += MPU_EMA_ALPHA * (mpu_gyroZ  - mpu_gyroZ_f);

  // Integrar Yaw (eje Z del giroscopio) para odometría angular
  mpu_yaw += mpu_gyroZ_f * dt;

  // ── Filtro complementario PITCH ───────────────────────────────────────────
  // pitch_accel = atan2(accelX, accelZ)  (rad → grados)
  // Convención: positivo = inclina hacia adelante del robot
  float pitch_accel_rad = atan2(mpu_accelX_f, mpu_accelZ_f);
  float pitch_accel_deg = pitch_accel_rad * (180.0f / PI);

  if (!mpu_pitch_initialized) {
    // Primer ciclo: inicializar con la lectura del acelerómetro directamente
    mpu_pitch = pitch_accel_deg;
    mpu_pitch_initialized = true;
  } else {
    // α=0.98: confía 98% en el giroscopio, corrige drift con 2% acelerómetro
    #ifdef HOVERBOARD_COMP_ALPHA
      const float alpha = HOVERBOARD_COMP_ALPHA;
    #else
      const float alpha = 0.98f;
    #endif
    mpu_pitch = alpha * (mpu_pitch + mpu_gyroY_f * dt) + (1.0f - alpha) * pitch_accel_deg;
  }

  // Debug continuo
  #ifdef ENABLE_MPU_DEBUG
  if (mpu_continuousDebug && (now - mpu_lastDebugOutput >= MPU_DEBUG_INTERVAL_MS)) {
    mpu_lastDebugOutput = now;
    Serial.print("[MPU] Yaw="); Serial.print(mpu_yaw, 2);
    Serial.print("° Gz="); Serial.print(mpu_gyroZ_f, 2);
    Serial.print("°/s Ax="); Serial.print(mpu_accelX_f, 3);
    Serial.print(" Ay="); Serial.print(mpu_accelY_f, 3);
    Serial.print(" Az="); Serial.print(mpu_accelZ_f, 3);
    Serial.print(" T="); Serial.print(mpu_tempC, 1);
    Serial.println("°C");
  }
  #endif
}

//===========================================================================
//========================== FUNCIONES DE CONSULTA ==========================
//===========================================================================

/** Devuelve el ángulo Yaw integrado en grados */
float mpu_getYaw()    { return mpu_yaw; }

/** Devuelve el ángulo Pitch (filtro complementario) en grados.
 *  Positivo = robot inclinado hacia adelante (accelX > 0) */
float mpu_getPitch()  { return mpu_pitch; }

/** Yaw relativo a la última referencia establecida */
float mpu_getRelativeYaw() { return mpu_yaw - mpu_yaw_reference; }

/** Establece la referencia de Yaw actual como 0° */
void mpu_setYawReference() {
  mpu_yaw_reference = mpu_yaw;
}

/** Resetea el ángulo Yaw integrado a 0 */
void mpu_resetYaw() {
  mpu_yaw = 0.0f;
  mpu_yaw_reference = 0.0f;
}

/** Velocidad angular Z filtrada en °/s */
float mpu_getGyroZ()  { return mpu_gyroZ_f; }

/** Aceleración filtrada en g */
float mpu_getAccelX() { return mpu_accelX_f; }
float mpu_getAccelY() { return mpu_accelY_f; }
float mpu_getAccelZ() { return mpu_accelZ_f; }

/** Temperatura en °C */
float mpu_getTemp()   { return mpu_tempC; }

/** True si el sensor respondió correctamente en I2C */
bool mpu_isReady()    { return mpu_initialized && mpu_found; }

//===========================================================================
//========================== IMPRESIÓN DE ESTADO ============================
//===========================================================================

void mpu_printStatus() {
  Serial.println("========== MPU9250/6500 STATUS ==========");
  if (!mpu_initialized) {
    Serial.println("  Estado: NO INICIALIZADO / NO DETECTADO");
    Serial.println("  Verifica conexión I2C (SDA=pin20, SCL=pin21)");
    Serial.println("=========================================");
    return;
  }
  Serial.print("  WHO_AM_I : 0x"); Serial.println(mpu_whoAmI, HEX);
  Serial.print("  Yaw      : "); Serial.print(mpu_yaw, 2); Serial.println(" °");
  Serial.print("  Yaw rel  : "); Serial.print(mpu_getRelativeYaw(), 2); Serial.println(" °");
  Serial.print("  Gyro Z   : "); Serial.print(mpu_gyroZ_f, 2); Serial.println(" °/s");
  Serial.print("  Accel    : X="); Serial.print(mpu_accelX_f, 3);
  Serial.print(" Y="); Serial.print(mpu_accelY_f, 3);
  Serial.print(" Z="); Serial.print(mpu_accelZ_f, 3); Serial.println(" g");
  Serial.print("  Temp     : "); Serial.print(mpu_tempC, 1); Serial.println(" °C");
  Serial.print("  Gyro off : X="); Serial.print(mpu_gyroOffsetX, 1);
  Serial.print(" Y="); Serial.print(mpu_gyroOffsetY, 1);
  Serial.print(" Z="); Serial.print(mpu_gyroOffsetZ, 1); Serial.println(" LSB");
  Serial.println("=========================================");
}

#endif  // ENABLE_MPU9250
#endif  // MPU9250_H
