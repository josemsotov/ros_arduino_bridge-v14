/**
 * BLUETOOTH_MODULE.H
 * Smart Golf Trolley — Módulo Bluetooth HC-05
 * Branch: feature/bluetooth
 *
 * HARDWARE:
 *   HC-05 conectado a Serial2 (Arduino Mega)
 *   TX2 = pin 16  →  RX del HC-05
 *   RX2 = pin 17  ←  TX del HC-05
 *   ENABLE = pin 40   HIGH = modo AT / LOW = modo datos
 *   STATE  = pin 42   HIGH = conectado / LOW = sin par
 *
 * PROTOCOLO:
 *   Mismo protocolo que Serial0 — todos los comandos funcionan por BT.
 *   El módulo hace bridge bidireccional BT ↔ Serial0.
 *
 * CONFIGURACIÓN AT (ejecutar UNA vez con BT_AT_CONFIG):
 *   AT+NAME=SmartTrolley
 *   AT+PSWD="1234"
 *   AT+UART=115200,0,0
 *   → Después de esto, cambiar BT_BAUD_RATE a 115200 en Configuration.h
 */

#ifndef BLUETOOTH_MODULE_H
#define BLUETOOTH_MODULE_H

#ifdef ENABLE_BLUETOOTH

//===========================================================================
//========================= ESTADO BT ======================================
//===========================================================================

bool bt_connected   = false;   // true cuando STATE=HIGH (HC-05 con par activo)
bool bt_initialized = false;
bool bt_atMode      = false;   // true mientras ENABLE=HIGH (modo AT)

// Buffer para comandos recibidos por BT
#define BT_CMD_BUFFER_SIZE 52
char     bt_cmdBuf[BT_CMD_BUFFER_SIZE];
uint8_t  bt_cmdLen = 0;
bool     bt_cmdReady = false;

//===========================================================================
//========================= INICIALIZACIÓN =================================
//===========================================================================

void bt_initialize() {
  pinMode(BT_ENABLE_PIN, OUTPUT);
  pinMode(BT_STATE_PIN,  INPUT);
  digitalWrite(BT_ENABLE_PIN, LOW);   // LOW = modo datos (normal)

  BT_SERIAL.begin(BT_BAUD_RATE);

  bt_initialized = true;
  bt_atMode      = false;

  Serial.print(F("[BT] HC-05 listo  Serial2 @ "));
  Serial.print(BT_BAUD_RATE);
  Serial.println(F(" baud | ENABLE=40 STATE=42"));
}

//===========================================================================
//========================= UPDATE =========================================
//===========================================================================

/**
 * Llamar desde loop(). Hace bridge bidireccional:
 *   BT → Serial0  (eco para monitor local)
 *   BT → commandBuffer (mismo procesador que USB)
 *   Serial0 → BT  (respuestas visibles en app BT)
 */
void bt_update() {
  if (!bt_initialized) return;

  // Actualizar estado de conexión
  bt_connected = (digitalRead(BT_STATE_PIN) == HIGH);

  // --- BT → procesar comandos ---
  while (BT_SERIAL.available()) {
    char c = (char)BT_SERIAL.read();

    if (bt_atMode) {
      // En modo AT: reenviar directo a Serial0 para que el usuario vea la respuesta
      Serial.write(c);
    } else {
      if (c == '\n' || c == '\r') {
        if (bt_cmdLen > 0) {
          bt_cmdBuf[bt_cmdLen] = '\0';
          bt_cmdReady = true;
          // Inyectar en el buffer compartido del Serial_Command_Processor
          extern String serialBuffer;
          extern bool   commandReady;
          serialBuffer = String(bt_cmdBuf);
          commandReady = true;
          bt_cmdLen    = 0;
        }
      } else if (bt_cmdLen < BT_CMD_BUFFER_SIZE - 1) {
        bt_cmdBuf[bt_cmdLen++] = c;
      }
    }
  }
}

//===========================================================================
//========================= COMANDOS AT ====================================
//===========================================================================

#ifdef ENABLE_BT_AT_COMMANDS

/**
 * Configuración automática del HC-05:
 *   - Nombre: SmartTrolley
 *   - PIN:    1234
 *   - Baud:   115200,0,0
 *
 * PROCEDIMIENTO:
 *   1. Apaga el HC-05
 *   2. Mantén pulsado el botón del HC-05 mientras enciendes (LED parpadea lento)
 *      — O conecta ENABLE=HIGH antes de encender el módulo —
 *   3. Envía el comando serial: BT_AT_CONFIG
 *   4. Espera "[BT] Config AT completa"
 *   5. Cambia BT_BAUD_RATE a 115200 en Configuration.h y recompila
 */
void bt_configAT() {
  Serial.println(F("[BT] Entrando en modo AT (ENABLE=HIGH)..."));
  digitalWrite(BT_ENABLE_PIN, HIGH);
  bt_atMode = true;
  delay(500);   // HC-05 necesita ~500 ms para entrar en AT

  // Reiniciar Serial2 a 38400 — velocidad AT por defecto del HC-05
  BT_SERIAL.end();
  BT_SERIAL.begin(38400);
  delay(100);

  // Secuencia de comandos AT
  struct { const char* cmd; const char* desc; } cmds[] = {
    { "AT",                   "Prueba comunicación" },
    { "AT+NAME=SmartTrolley", "Nombre del dispositivo" },
    { "AT+PSWD=\"1234\"",     "PIN de emparejamiento" },
    { "AT+UART=115200,0,0",  "Velocidad → 115200" },
    { "AT+RESET",             "Reiniciar módulo" }
  };

  for (auto& c : cmds) {
    Serial.print(F("[BT] >> ")); Serial.println(c.cmd);
    BT_SERIAL.println(c.cmd);
    delay(300);
    // Mostrar respuesta
    while (BT_SERIAL.available()) {
      Serial.write(BT_SERIAL.read());
    }
    Serial.println();
  }

  Serial.println(F("[BT] Config AT completa."));
  Serial.println(F("[BT] Cambia BT_BAUD_RATE a 115200 en Configuration.h y recompila."));

  // Volver a modo datos
  digitalWrite(BT_ENABLE_PIN, LOW);
  bt_atMode = false;
  BT_SERIAL.end();
  BT_SERIAL.begin(BT_BAUD_RATE);   // baud configurado en Pins.h
}

#endif // ENABLE_BT_AT_COMMANDS

//===========================================================================
//========================= STATUS =========================================
//===========================================================================

void bt_printStatus() {
  Serial.println(F("=== Bluetooth HC-05 ==="));
  Serial.print(F("  Inicializado : ")); Serial.println(bt_initialized ? F("SI") : F("NO"));
  Serial.print(F("  Conectado    : ")); Serial.println(bt_connected    ? F("SI (par activo)") : F("NO"));
  Serial.print(F("  Modo AT      : ")); Serial.println(bt_atMode       ? F("SI") : F("NO"));
  Serial.print(F("  Baud rate    : ")); Serial.println(BT_BAUD_RATE);
  Serial.print(F("  Serial       : Serial2  TX2=16  RX2=17"));
  Serial.println();
  Serial.print(F("  ENABLE pin   : ")); Serial.println(BT_ENABLE_PIN);
  Serial.print(F("  STATE pin    : ")); Serial.println(BT_STATE_PIN);
  Serial.println(F("========================="));
}

#endif // ENABLE_BLUETOOTH
#endif // BLUETOOTH_MODULE_H


#ifndef BLUETOOTH_MODULE_H
#define BLUETOOTH_MODULE_H

#ifdef ENABLE_BLUETOOTH

//===========================================================================
//========================= ESTADO BT ======================================
//===========================================================================

bool bt_connected    = false;   // true cuando STATE=HIGH (HC-05 con par activo)
bool bt_initialized  = false;

//===========================================================================
//========================= INICIALIZACIÓN =================================
//===========================================================================

void bt_initialize() {
  // Configurar pines de control
  pinMode(BT_ENABLE_PIN, OUTPUT);
  pinMode(BT_STATE_PIN,  INPUT);

  // ENABLE=LOW → modo datos (comunicación normal)
  digitalWrite(BT_ENABLE_PIN, LOW);

  // Iniciar Serial2 a la velocidad configurada
  BT_SERIAL.begin(BT_BAUD_RATE);

  bt_initialized = true;

  Serial.print(F("[BT] HC-05 inicializado en Serial2 @ "));
  Serial.print(BT_BAUD_RATE);
  Serial.println(F(" baud"));
  Serial.println(F("[BT] ENABLE=pin40  STATE=pin42"));
}

//===========================================================================
//========================= UPDATE (llamar desde loop) =====================
//===========================================================================

void bt_update() {
  if (!bt_initialized) return;

  // Actualizar estado de conexión leyendo pin STATE
  bt_connected = (digitalRead(BT_STATE_PIN) == HIGH);

  // Reenviar datos BT → Serial principal (para debug)
  while (BT_SERIAL.available()) {
    char c = BT_SERIAL.read();
    Serial.write(c);           // espejo en Serial0 para debug
    // TODO: parsear comandos del mismo protocolo que Serial_Command_Processor
  }
}

//===========================================================================
//========================= COMANDOS AT ====================================
//===========================================================================

#ifdef ENABLE_BT_AT_COMMANDS
/**
 * Entra en modo AT (ENABLE=HIGH, reiniciar módulo con power cycle previo)
 * Solo para configuración inicial del HC-05.
 * Llamar desde Serial_Command_Processor con comando "BT_AT"
 */
void bt_enterAT() {
  digitalWrite(BT_ENABLE_PIN, HIGH);
  delay(100);
  BT_SERIAL.println("AT");    // Prueba básica — HC-05 responde "OK"
  Serial.println(F("[BT] Modo AT activo. Envía comandos AT por BT_SERIAL."));
}

void bt_exitAT() {
  digitalWrite(BT_ENABLE_PIN, LOW);
  Serial.println(F("[BT] Modo AT desactivado."));
}
#endif // ENABLE_BT_AT_COMMANDS

//===========================================================================
//========================= STATUS =========================================
//===========================================================================

void bt_printStatus() {
  Serial.println(F("=== Bluetooth HC-05 ==="));
  Serial.print(F("  Inicializado : ")); Serial.println(bt_initialized ? F("SI") : F("NO"));
  Serial.print(F("  Conectado    : ")); Serial.println(bt_connected    ? F("SI") : F("NO"));
  Serial.print(F("  Baud rate    : ")); Serial.println(BT_BAUD_RATE);
  Serial.print(F("  ENABLE pin   : ")); Serial.println(BT_ENABLE_PIN);
  Serial.print(F("  STATE pin    : ")); Serial.println(BT_STATE_PIN);
}

#endif // ENABLE_BLUETOOTH
#endif // BLUETOOTH_MODULE_H
