# Handover — Smart Trolley

Actualizado: 2026-07-23

## Estado actual

- Plataforma en uso: Raspberry Pi 5. El usuario identifica la placa como 8 GB, pero una medición remota anterior mostró aproximadamente 1.9 GiB visibles para Linux y uso elevado de swap. Debe verificarse antes de atribuir problemas a falta de hardware.
- El sistema mantiene los modos Stadia, follower y control desde la interfaz. Stadia debe ser el modo predeterminado hasta que el usuario seleccione otro modo.
- La cámara usada por la aplicación follower es Kinect/USB. Durante pruebas de conexión Ethernet, el vídeo tardaba en aparecer y después se congelaba.
- La aplicación actual no tiene integración confirmada con Coral Edge TPU. La inferencia observada utiliza MediaPipe/OpenCV sobre CPU; conectar el Coral no la acelera automáticamente.
- La migración a Coral o AI HAT queda pospuesta. La prioridad acordada es seguir estabilizando el sistema actual.

### Cierre operativo 2026-07-23

- El Pi está accesible por Wi-Fi mediante `josemsotov@192.168.40.52`. Ethernet era `192.168.40.32`.
- El robot quedó detenido: follower deshabilitado, estado `WAITING`, `cmd_vel=0`, PWM izquierdo/derecho `0/0` y RPM `0/0`.
- Stadia quedó como modo predeterminado y vuelve a controlar el robot.
- Se produjo un incidente durante pruebas: el robot continuó avanzando porque varios nodos publicaban simultáneamente en `/cmd_vel`. Stadia podía seguir publicando a 20 Hz mientras follower estaba activo, impidiendo que venciera el timeout del firmware.
- No realizar nuevas pruebas con las ruedas apoyadas hasta validar los cuatro casos de fail-safe indicados abajo.

## Fail-safes desplegados después del incidente

- `robot_operator_web` ahora arbitra modos de manera exclusiva:
  - `FOLLOWER` pausa Stadia antes de habilitar follower.
  - `STADIA` deshabilita follower y publica STOP antes de devolver el control.
  - `IDLE`, `GESTURE` y STOP deshabilitan follower, detienen y apagan la salida Stadia.
- `stadia_node` publica STOP al desconectarse el mando.
- Al volver a Stadia, se relee la posición física de los ejes para no reutilizar un valor antiguo.
- `stadia_node` escucha `/follower/enable`: cualquier activación de follower desde interfaz, gesto o ROS pausa inmediatamente la salida Stadia.
- `arduino_bridge` incorpora `cmd_timeout=0.50`; si deja de recibir `/cmd_vel` después de una orden no nula, envía `v 0.0 0.0`.
- Paquetes recompilados con:
  - `colcon build --merge-install --symlink-install --packages-select arduino_bridge_ros2 robot_operator_web`
- Copias exactas desplegadas guardadas localmente en `.codex_runtime_fix/deployed_20260723/`.
- Scripts reproducibles del hotfix:
  - `.codex_runtime_fix/apply_cmd_vel_failsafe.py`
  - `.codex_runtime_fix/apply_stadia_follower_interlock.py`

## Mejoras aplicadas en el Pi durante la sesión

- Stream MJPEG persistente para RGB mediante `/api/stream/rgb.mjpg`.
- Interfaz configurada para consumir el stream persistente; para evitar caché antigua se utilizó `http://robot.local:8080/static/touch.html?v=20260723-mjpeg`.
- RGB limitado aproximadamente a 4 FPS y profundidad a 1.3 FPS, con ancho de 480 px y menor calidad JPEG.
- MediaPipe de palma abierta limitado aproximadamente a 5 FPS.
- `STOP` del follower cancela también el enrolamiento de identidad.
- Procesamiento de profundidad Kinect vectorizado y omitido cuando follower está deshabilitado.
- Recuperación automática de Kinect ante pérdida USB.
- Seguimiento mediante Supervision/ByteTrack, con MediaPipe Pose/HOG como alternativas de detección.

Estas mejoras fueron aplicadas y probadas sobre la instalación del Pi en una sesión anterior. Las copias `.codex_*` locales deben compararse con el Pi antes de considerarlas la fuente desplegada definitiva.

## Mediciones relevantes

- Antes de optimizar, `follower_node` consumía aproximadamente 42–47 % de un núcleo; después de vectorizar profundidad quedó cerca de 8 % en la medición observada.
- En reposo, la carga agregada del Pi quedó alrededor de 29 %, con aproximadamente 70.6 % de CPU libre.
- Procesos observados en una muestra: `open_palm_node` ~58 %, `web_server` ~33 %, `kinect_node` ~25 % y `follower_node` ~8 %. Son lecturas instantáneas, no límites garantizados.

## Problemas abiertos

1. Confirmar que el stream permanezca fluido durante varios minutos y después de desconectar/reconectar Ethernet.
2. Confirmar que Kinect se recupere sin reiniciar servicios después de una pérdida USB o de red.
3. Probar el mando Stadia desde la interfaz existente y verificar movimiento, parada y prioridad entre modos.
4. Verificar la memoria real del Pi con `free -h`, `/proc/meminfo`, `/proc/cmdline` y `vcgencmd get_config total_mem`.
5. Confirmar si el Coral aparece en `lsusb` y qué runtime está instalado. No modificar aún la inferencia para utilizarlo.
6. Revisar autenticación SSH: `robot.local` responde, pero el intento desde esta sesión usó el usuario local `cools` y fue rechazado. Hace falta usar el usuario/clave configurados para el Pi.

## Próxima prueba recomendada

1. Elevar las ruedas y mantener disponible el paro físico.
2. Validar Stadia → follower y confirmar que solo queda un productor efectivo de movimiento.
3. Validar follower → Stadia y confirmar STOP intermedio.
4. Validar desconexión del mando durante una orden: debe publicar cero inmediatamente.
5. Validar el watchdog de `arduino_bridge`: una orden única debe caer a cero en menos de un segundo.
6. Solo después, probar avance, retroceso y giro a velocidad reducida.
7. Corregir el cliente antiguo que todavía solicita `/api/frame/rgb.jpg` y `/api/frame/depth.jpg`; esa carga puede saturar la interfaz.
8. Implementar los endpoints `/api/identity/*` que el HTML ya intenta utilizar.
9. Registrar CPU, RAM, swap y logs durante cualquier congelación.

## Seguridad

- Hacer las primeras pruebas de movimiento con las ruedas motrices elevadas.
- Mantener accesible el paro físico y el STOP de la interfaz.
- No probar follower hasta confirmar que cambiar a Stadia o pulsar STOP cancela sus órdenes.

## Respaldo asociado

- Respaldo final, incluyendo este índice y handover: `BACKUPS/SMART_TROLLEY_WORKSPACE_FINAL_20260723_021033.tar.gz`.
- Respaldo previo a actualizar la documentación: `BACKUPS/SMART_TROLLEY_WORKSPACE_20260723_020932.tar.gz`.
- Respaldo del runtime del Pi después de los fail-safes: `BACKUPS/pre_tomorrow_20260723_035630.tar.gz`.
- SHA-256 del respaldo del Pi: `9cee5e4e3dba005eee39347ea1145a28c04faa6d929f6826781cf3c5843bba66`.
- Cada archivo tiene su verificación SHA-256 en el archivo homónimo terminado en `.sha256`.
