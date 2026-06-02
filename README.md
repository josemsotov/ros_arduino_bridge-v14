# MOTOR-INTERFACE-V-13

Sistema de control de motores para Smart Golf Trolley con integración ROS2.

## 📁 Estructura del Proyecto

```
MOTOR_INTERFACE-V-13/
├── MOTOR-INTERFACE-V-13.ino    # Archivo principal
├── Configuration.h              # Configuración del sistema
├── Pins.h                       # Definición de pines
├── Modules.h                    # Inclusión de módulos
├── Core_Functions.h             # Funciones principales
├── Motor_Control.h              # Control de motores
├── Robot_States.h               # Máquina de estados
├── Hall_Sensors.h               # Sensores Hall
├── pid_control.h                # Control PID
├── ROS2_Bridge.h                # Comunicación ROS2
├── Serial_Command_Processor.h   # Procesador de comandos
└── Joystick_Module.h            # Módulo de joystick
```

## 🚀 Comandos Disponibles

### Comandos Locales
- `HELP` - Mostrar ayuda
- `INFO` - Información del sistema
- `HABILITAR` / `INHABILITAR` - Control de estado
- `ADELANTE` / `ATRAS` / `STOP` - Control manual
- `PTEST` - Prueba PWM=50 por 2s
- `VTEST` - Prueba PWM=30 continua

### Comandos ROS2
- `v <linear> <angular>` - Comando de velocidad
- `e` - Request encoder counts
- `r` - Reset encoders
- `s` - Request status
- `c` - Calibración PID

## ⚙️ Configuración

Edita `Configuration.h` para habilitar/deshabilitar funcionalidades:
- `ENABLE_SERIAL_COMMANDS` - Comandos por serial
- `ENABLE_ROS2_BRIDGE` - Integración ROS2
- `ENABLE_HALL_SENSORS` - Sensores Hall
- `ENABLE_PID_CONTROL` - Control PID
- `ENABLE_JOYSTICK` - Control por joystick

## 📡 Comunicación Serial

- **Baud Rate:** 115200
- **Protocolo:** Comandos de texto ASCII
- **Compatible con:** ROS2 Humble (Raspberry Pi 5)

## 🔧 Hardware

- **Placa:** Arduino Mega 2560
- **Motores:** Driver L298N x2
- **Sensores:** Hall effect x2
- **Joystick:** Analógico 2 ejes

## 📝 Notas

- Todos los cambios futuros se realizarán en el mismo archivo `.ino`
- No se generarán archivos de documentación/debug adicionales
- Los backups se mantienen solo durante desarrollo activo

---

**Autor:** JMS 2025  
**Versión:** 13
