# Índice del proyecto — Smart Trolley V14

Actualizado: 2026-07-23

## Documentos principales

- `CODEBASE_MEMORY.md`: mapa canónico de arquitectura, módulos y reglas de desarrollo.
- `HANDOVER.md`: estado operativo, cambios recientes, pruebas realizadas y próximos pasos.
- `README.md`: descripción general del firmware y uso básico.
- `GEMINI.md`: reglas de navegación y mantenimiento de contexto del repositorio.

## Código activo local

- `MOTOR-INTERFACE-V14.ino`: entrada principal del firmware Arduino Mega 2560.
- `Configuration.h`: funciones habilitadas y parámetros globales.
- `Modules.h`: composición y orden de módulos.
- `Motor_Control.h`: actuación y seguridad de motores.
- `Robot_States.h`: modos, estados y maniobras.
- `ROS2_Bridge.h`: protocolo serie con el Raspberry Pi.
- `Serial_Command_Processor.h`: comandos locales y despacho.
- `stadia_controller.py`: utilidad local para el mando Stadia.

## Aplicación Raspberry Pi conservada en el workspace

- `.codex_gesture_update/robot_follower/`: nodos y configuración del follower.
- `.codex_gesture_update/robot_operator_web/`: interfaz web y streaming.
- `.codex_gesture_update/start_robot_follower.sh`: arranque de la aplicación.
- `.codex_touch_mode/`, `.codex_touch_edit/`, `.codex_stadia_edit/`: copias auxiliares de cambios de interfaz y modos.

Estos directorios `.codex_*` son copias de trabajo y no garantizan por sí solos que coincidan con la instalación activa del Pi. Antes de desplegar, comparar con el workspace ROS2 remoto.

## Respaldo y artefactos

- `BACKUPS/`: respaldos fechados del PC y del Pi.
- `build_output/`: binarios generados; no es fuente canónica.
- `MOTOR-INTERFACE-V-13/`: versión histórica; no editar para cambios de V14.

## Orden recomendado de lectura

1. `HANDOVER.md`
2. `CODEBASE_MEMORY.md`
3. El archivo fuente concreto indicado por ambos documentos

