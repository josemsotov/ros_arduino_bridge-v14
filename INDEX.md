# Índice del proyecto — Smart Trolley V14

Actualizado: 2026-07-23

## Lectura inicial

1. `HANDOVER.md`: estado operativo, seguridad, última prueba y próximo paso.
2. `CODEBASE_MEMORY.md`: arquitectura y reglas del repositorio.
3. `README.md`: firmware y uso general.

## Código principal

- `MOTOR-INTERFACE-V14.ino`: firmware Arduino Mega 2560.
- `Configuration.h`: configuración global.
- `Modules.h`: composición de módulos.
- `Motor_Control.h`: motores y seguridad.
- `Robot_States.h`: modos y estados.
- `ROS2_Bridge.h`: protocolo serie Raspberry Pi–Arduino.
- `Serial_Command_Processor.h`: comandos locales.
- `stadia_controller.py`: utilidad del mando.

## Snapshot activo del Raspberry Pi

- `.codex_runtime_fix/pi8_safety_net/current/`: follower, Stadia, bridge y parámetros.
- `.codex_runtime_fix/pi8_safety_net/web_current/`: servidor e interfaz web.
- `.codex_runtime_fix/pi8_safety_net/`: pruebas, límites y paro de emergencia.
- `.codex_runtime_fix/pi8_face_static/`: implementación y utilidades de identidad facial.

Las capturas de cámara y cachés Python no forman parte del respaldo.

## Historial y artefactos

- `.codex_runtime_fix/deployed_20260723/`: copias de despliegues anteriores.
- `.codex_gesture_update/`: follower e interfaz de una etapa anterior.
- `.codex_touch_mode/`, `.codex_touch_edit/`, `.codex_stadia_edit/`: cambios históricos.
- `BACKUPS/`: archivos de respaldo fechados.
- `MOTOR-INTERFACE-V-13/`: versión histórica, no editar para V14.
