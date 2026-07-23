# Handover — Smart Trolley V14

Actualizado: 2026-07-23

## Estado operativo actual

- Hardware activo: Raspberry Pi 5 de 8 GB.
- Acceso Ethernet verificado: `josemsotov@192.168.40.73`.
- Servicios de usuario activos:
  - `robot-follower.service`
  - `robot-operator-web.service`
- Interfaz/API: `http://192.168.40.73:8080`.
- El modo seguro predeterminado es `STADIA`.
- Estado al cerrar: follower deshabilitado, `/cmd_vel=0`, PWM `0/0` y RPM `0/0`.
- El robot permanece suspendido durante las pruebas de ruedas.

## Funcionalidad desplegada

### Arbitraje y safety net

- Stadia toma el control automáticamente al conectarse por Bluetooth.
- El mando debe observarse con las palancas centradas antes de quedar armado.
- Stadia, OFF o desconexión cancelan cualquier autorización anterior del follower.
- Solo `/follower/authorized_enable` puede autorizar el follower después de seleccionar
  intencionalmente ese modo.
- Órdenes directas, gestos y `/follower/enable` se rechazan mientras STADIA/OFF tenga prioridad.
- El follower exige una sesión facial válida antes de arrancar.
- Al detener el follower se restaura Stadia y se publica STOP.
- El firmware dispone de watchdog de órdenes con `cmd_timeout=0.5`.
- Paro de emergencia reproducible:
  `.codex_runtime_fix/pi8_safety_net/emergency_stadia_stop.sh`.

### Identidad, cuerpo y LiDAR

- La prueba facial estática detecta, enrola y verifica el rostro sin mover motores.
- El cuerpo se sigue con el detector/track de la aplicación follower.
- La asociación cuerpo–LiDAR agrupa puntos contiguos y selecciona el clúster usando
  alineación visual, continuidad de distancia y tamaño.
- Se rechazan saltos de objetivo mayores de `0.60 m`.
- Distancia y ángulo aceptados se suavizan.
- Telemetría añadida:
  - `lidar_raw_target_dist`
  - `lidar_raw_target_angle`
  - `lidar_cluster_count`
  - `lidar_target_status`
- El modo `FACE_STATIC_DRY_RUN` calcula las órdenes previstas, pero fuerza movimiento real cero.

### Interfaz y telemetría

- `robot_operator_web` escucha el `/cmd_vel` real del bus ROS.
- La API expone sus valores y antigüedad.
- La interfaz muestra comando real, objetivo LiDAR crudo/filtrado, clústeres y estado.

## Última prueba: cluster dry-run

Resultado: aprobado, sin movimiento.

- Identidad facial verificada: puntuación `0.826`.
- Tracking corporal activo: confianza `0.884`.
- Objetivo LiDAR: aproximadamente `2.426 m`, clúster de 14 puntos.
- En la muestra completa, la distancia cruda permaneció entre `2.4095–2.495 m`.
- No hubo rechazos por salto; el estado permaneció `tracking`.
- Orden prevista del follower: `0.069 m/s`, `-0.081 rad/s`.
- `/cmd_vel` real: `0.000 m/s`, `0.000 rad/s`.
- PWM y RPM: cero.

Una lectura inicial mostró `no_face` porque el rostro estaba girado/inclinado. Al mirar
hacia la cámara, la verificación se completó automáticamente. La cámara, el tracking
corporal y el LiDAR sí funcionaban.

## Fuente respaldada

- Follower activo: `.codex_runtime_fix/pi8_safety_net/current/follower_node.py`
- Stadia activo: `.codex_runtime_fix/pi8_safety_net/current/stadia_node.py`
- Bridge activo: `.codex_runtime_fix/pi8_safety_net/current/arduino_node.py`
- Parámetros: `.codex_runtime_fix/pi8_safety_net/current/follower_params.yaml`
- Servidor web: `.codex_runtime_fix/pi8_safety_net/web_current/web_server.py`
- Interfaz: `.codex_runtime_fix/pi8_safety_net/web_current/index.html`
- Scripts de prueba y paro: `.codex_runtime_fix/pi8_safety_net/`
- Desarrollo de prueba facial: `.codex_runtime_fix/pi8_face_static/`

Las capturas de cámara y los archivos `__pycache__` se excluyen del respaldo GitHub.

## Próximo paso

1. Mantener el robot suspendido y el paro físico accesible.
2. Repetir una prueba corta con rostro frontal y confirmar identidad + clúster estable.
3. Seleccionar FOLLOWER intencionalmente desde Stadia.
4. Autorizar una prueba de ruedas limitada, vigilando simultáneamente:
   `/cmd_vel`, PWM, RPM, identidad, body track y LiDAR.
5. Confirmar parada por pérdida de rostro, pérdida de objetivo, STOP y takeover de Stadia.
6. No apoyar las ruedas hasta aprobar todos los casos anteriores.

## Comandos útiles

```text
ssh josemsotov@192.168.40.73
systemctl --user status robot-follower.service robot-operator-web.service
```
