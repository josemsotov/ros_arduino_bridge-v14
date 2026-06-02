# Changelog — arduino_bridge

Formato: `vMAJOR.MINOR`  
- **MAJOR** se incrementa automáticamente en cada `git commit` (hook pre-commit)  
- **MINOR** se incrementa manualmente durante el desarrollo entre commits

---

## v1.0 — Sensor Fusion & Operation Modes
**Commit inicial del esquema de versiones**

- Renombrado sketch a `arduino_bridge`
- Sistema de modos de operación: `MODO_BANCO_ESTATICO` / `MODO_BANCO_DINAMICO` / `MODO_OPERATIVO`
- Odometría con fusión sensorial Hall + MPU9250 (filtro complementario α=0.98)
- Contadores Hall firmados (`leftHallSigned`, `rightHallSigned`) — dirección por flags software
- Forward declarations para resolver dependencias cruzadas `Hall_Sensors.h` ↔ `Odometry.h`
- Protocolo ROS2 Bridge: comandos `v`, `e`, `o`, `r`, `s`
- Corrección ZS-X11H: secuencia STOP=LOW → STOP=FLOAT → DIR → PWM
- Direcciones verificadas en EEPROM: IZQ=HIGH, DER=LOW
- Serial boot muestra versión y modo activo

---

<!-- Plantilla para próximas entradas:

## v2.0 — <Título corto de la integración>
**Descripción breve**

- Cambio 1
- Cambio 2

-->
