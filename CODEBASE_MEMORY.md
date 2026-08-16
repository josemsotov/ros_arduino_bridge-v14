# Codebase Memory — MOTOR-INTERFACE-V14

> Canonical spatial map for the active workspace. Read this file before broad repository exploration. Validate details against the smallest relevant set of source files when making changes.

> Estado operativo y continuidad: consultar también `INDEX.md` y `HANDOVER.md` (actualizados el 2026-07-23).

## Arquitectura y Flujo

### Visión general

El proyecto controla un Smart Golf Trolley de tracción diferencial mediante firmware C++/Arduino para una **Arduino Mega 2560**. Sigue un diseño modular inspirado en Marlin: el sketch principal incluye configuración, pines y un agregador de módulos; `#define` y `#ifdef` determinan qué capacidades se compilan. No existe un contenedor de inyección de dependencias ni separación por librerías compiladas: los módulos son headers con estado global y funciones, y el **orden de inclusión en `Modules.h` forma parte de la arquitectura**.

El segundo bloque del proyecto son utilidades Python independientes para control, diagnóstico, calibración, simulación y visualización. Se comunican con el firmware mediante un protocolo ASCII sobre puerto serie a 115200 baudios. El llamado “ROS2 Bridge” del firmware es un adaptador de protocolo serie compatible con un nodo externo en Raspberry Pi 5. La raíz no contiene un workspace ROS2 activo completo; existen copias auxiliares de paquetes del Pi bajo `.codex_gesture_update/`, pero deben compararse con la instalación remota antes de usarlas como fuente de despliegue.

### Tecnologías principales

- Arduino C++/AVR, Arduino core y `Wire` para I2C.
- Arduino Mega 2560, PWM directo mediante Timer5 y drivers de motor.
- Sensores Hall, IMU MPU9250/MPU6500 y sensores de corriente ACS712.
- Python 3 para herramientas host; dependencias observadas: `pyserial`, `tkinter`, `matplotlib` y, para controladores HID concretos, `pywinusb`.
- Arduino CLI para compilar/subir mediante `upload.ps1` o `upload.bat`.

### Patrón de diseño

- **Superloop embebido:** `setup()` inicializa subsistemas una vez y `loop()` los actualiza cooperativamente sin scheduler.
- **Feature flags de compilación:** `Configuration.h` habilita módulos; `Modules.h` resuelve su inclusión y orden.
- **Máquina de estados:** `Robot_States.h` modela estados, desplazamientos y giros no bloqueantes.
- **Capas funcionales ligeras:** entrada serial/joystick → interpretación y estado/control → actuadores; sensores realimentan control, seguridad y telemetría.
- **Fail-safe:** límites de PWM, timeouts, comprobaciones periódicas, frenado/parada y detección de inclinación protegen el hardware.

### Ciclo de vida y flujo extremo a extremo

1. Al arrancar, `MOTOR-INTERFACE-V14.ino::setup()` abre `Serial`, llama a `initializeSystem()`, inicializa el bridge serie ROS2 y los sensores de corriente, y activa el modo hoverboard cuando IMU y balance están habilitados.
2. `initializeSystem()` configura pines/motores, sensores habilitados y estado seguro inicial. La IMU se detecta, configura y puede autocalibrarse.
3. En cada iteración, `readSerialCommands()` acumula una línea ASCII y `processSerialCommand()` la enruta. Los comandos ROS2 (`v`, `e`, `r`, `s`, etc.) se delegan primero a `ROS2_Bridge.h`; los comandos locales modifican estados, pruebas, calibraciones o actuadores.
4. Un comando de velocidad se limita, se convierte en consignas diferenciales y termina en funciones de `Motor_Control.h`, que escriben dirección, freno/STOP y PWM de Timer5. Los comandos discretos pasan normalmente por `Robot_States.h`.
5. En paralelo, `mpu_update()`, `current_sensors_update()` y `updateHallSpeeds()` actualizan inclinación, corriente, pulsos y velocidad. `hoverboard_update()` usa pitch/giroscopio y vuelve a inyectar una consigna corregida en el flujo ROS2. El PID está disponible como módulo, aunque `ENABLE_PID_CONTROL` está desactivado en la configuración actual.
6. `updateCrossing()` y `updatePruebaTotal()` avanzan operaciones no bloqueantes; `systemMonitoring()` y `motorSafetyCheck()` aplican seguridad. `ros2_update()` vigila el timeout de comandos y publica/responde telemetría por serie.
7. En el host, cada script Python abre el puerto serie, envía el mismo protocolo ASCII y procesa respuestas. Las GUI desacoplan lectura serial y refresco visual mediante threads/colas.

## Árbol de Directorios Simplificado

```text
MOTOR-INTERFACE-V14/
├── MOTOR-INTERFACE-V14.ino       # Entry point y superloop del firmware activo
├── Configuration.h / Pins.h      # Feature flags, constantes y mapeo físico
├── Modules.h                     # Grafo y orden de inclusión de módulos
├── *_Control.h, *_Sensors.h      # Actuación, control y adquisición
├── Robot_States.h                # Máquina de estados y maniobras
├── ROS2_Bridge.h                 # Protocolo serie para integración externa
├── Serial_Command_Processor.h    # Parser y dispatcher de comandos locales
├── *.py                          # GUI, control, calibración, diagnóstico y tests host
├── TEST_*/                       # Sketches Arduino aislados para pruebas de hardware
├── MOTOR-INTERFACE-V-13/         # Snapshot histórico; no editar como implementación V14
├── build_output/                 # Binarios generados; no es fuente
├── BACKUPS/                      # Archives/bundles de recuperación; no es fuente activa
├── .codex_*/                     # Artefactos auxiliares de sesiones/herramientas; no canónicos
└── .github/                      # Configuración e instrucciones de GitHub/Copilot
```

Los archivos Python de raíz son ejecutables independientes, no un paquete único. Los más generales son `robot_interface.py` (GUI de control), `robot_sim.py` (simulación/control), `motor_monitor.py` y `balance_dashboard.py` (telemetría); `diag_*.py`, `test_*.py`, `run_motor.py`, `manual_hall_test.py` y `calibrate_kpv.py` son herramientas especializadas.

## Puntos de Entrada y Configuración

- **Firmware:** `MOTOR-INTERFACE-V14.ino`, concretamente `setup()` y `loop()`.
- **Configuración funcional:** `Configuration.h`. Estado actual relevante: Hall, MPU, hoverboard, ACS712, bridge ROS2 y comandos seriales habilitados; joystick y PID general deshabilitados.
- **Configuración física:** `Pins.h`. Es la fuente de verdad para pines y restricciones del Timer5; cambios aquí requieren validación sobre Arduino Mega 2560.
- **Composición:** `Modules.h`. Mantener el orden documentado: motores → estados → sensores/IMU → PID → ROS2 → balance → joystick → core → comandos seriales.
- **Constantes locales críticas:** algunos límites siguen junto a su dominio: PWM/timeout en `Motor_Control.h`, geometría/PPR en `Robot_States.h`, límites de protocolo en `ROS2_Bridge.h`, filtro IMU en `MPU9250.h` y balance en `Balance_Controller.h`/`Configuration.h`.
- **Carga:** `upload.ps1` y `upload.bat` invocan Arduino CLI para Mega 2560 y actualmente contienen rutas/puerto locales (por ejemplo `COM4`).
- **Herramientas Python:** normalmente `main()` o bloque `if __name__ == "__main__"`; varias aceptan el puerto como argumento y otras conservan `COM4` como valor local.
- **Variables de entorno:** no se detectó `.env` ni configuración basada en environment variables. La configuración está compilada en headers, scripts de carga y argumentos/constantes de los scripts Python.

## Mapa de Conexiones y Dependencias

- `MOTOR-INTERFACE-V14.ino` → `Configuration.h`, `Pins.h`, `Modules.h`.
- `Modules.h` → todos los módulos activos y define su orden de visibilidad.
- `Core_Functions.h` → inicialización de motores/sensores, estado del sistema y rutinas de seguridad.
- `Serial_Command_Processor.h` → `ROS2_Bridge.h` para comandos de protocolo; → `Robot_States.h`, `Motor_Control.h`, PID, IMU, balance y diagnósticos para comandos locales.
- `ROS2_Bridge.h` → sensores Hall/estado de motores para feedback; → `Motor_Control.h` y PID por rueda para convertir `cmd_vel` en actuación diferencial.
- `Balance_Controller.h` → `MPU9250.h` para pitch/gyro; → variables/funciones de `ROS2_Bridge.h` para añadir corrección de velocidad.
- `Robot_States.h` → `Motor_Control.h` y contadores Hall para movimientos, cruces y parada.
- `Hall_Sensors.h` → interrupciones y geometría/PPR para medir movimiento; alimenta estado, ROS2 y control.
- `Current_Sensors.h` → ADC/pines ACS712; aporta monitoreo de consumo.
- `Joystick_Module.h` → control diferencial de motores cuando la feature está activa.
- Scripts Python → `pyserial` → puerto USB serie → parser firmware → control/telemetría. Las GUI además dependen de Tkinter/Matplotlib; los scripts Stadia/HID pueden depender de `pywinusb`.

## Reglas de Desarrollo

- Mantener compatibilidad con Arduino AVR/Mega 2560: memoria limitada, sin excepciones C++ ni asignaciones dinámicas innecesarias en el hot path.
- Usar guards de inclusión (`#ifndef`/`#define`/`#endif`) y proteger módulos opcionales con los mismos feature flags de `Configuration.h`.
- Respetar estrictamente el orden de `Modules.h`; estos headers comparten símbolos globales y no son unidades independientes.
- Nombres observados: macros/constantes de preprocesador en `UPPER_SNAKE_CASE`; funciones y variables C++ mayoritariamente `camelCase` o prefijos de subsistema (`ros2_`, `mpu_`, `joy_`); clases Python en `PascalCase`, funciones/variables Python en `snake_case`.
- Preferir tipos explícitos adecuados al hardware (`uint8_t`, `uint16_t`, `unsigned long`, `float`) y referencias para parámetros de salida en C++.
- Mantener `loop()` no bloqueante salvo retardos mínimos o secuencias de inicialización/calibración justificadas; temporizar tareas periódicas con `millis()`.
- Todo comando externo debe validar formato, limitar rangos con `constrain`/equivalente y caer a un estado seguro ante timeout, sensor inválido o entrada desconocida.
- No sustituir la escritura PWM especializada de Timer5 por `analogWrite()` sin revisar `Pins.h` y `Motor_Control.h`; el código documenta efectos sobre registros `COM5x`.
- Usar las macros `DEBUG_PRINT*` y `HELP_COMMAND` para salida condicional; vigilar RAM/flash al añadir strings o debug.
- En Python, cerrar el puerto serie de forma segura, tolerar desconexiones/timeouts y evitar actualizar Tkinter directamente desde threads de lectura.
- Tratar `MOTOR-INTERFACE-V-13/`, `BACKUPS/`, `build_output/`, `.venv/`, `__pycache__/` y `.codex_*/` como históricos, generados o auxiliares salvo que una tarea los nombre expresamente.
- Antes de cambiar hardware, protocolo serial, estados, orden modular o configuración transversal, revisar las dependencias anteriores y actualizar este mapa.
