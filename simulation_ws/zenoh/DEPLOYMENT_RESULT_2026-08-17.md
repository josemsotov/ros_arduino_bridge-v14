# Resultado de despliegue Zenoh persistente

## Estado

- Router Zenoh TCP: `192.168.40.74:7447`.
- Servicio `smart-trolley-zenoh-router.service`: habilitado y activo.
- `robot-follower.service`: activo con `rmw_zenoh_cpp`.
- `robot-operator-web.service`: activo con `rmw_zenoh_cpp`.
- RViz en WSL: conectado como cliente TCP.

## Pruebas aprobadas

- Talker ARM64 a listener AMD64 a traves de NAT.
- Descubrimiento de Arduino, Stadia, follower, LiDAR, Kinect y web.
- LiDAR: aproximadamente 9.6--9.8 Hz.
- Kinect PointCloud2: aproximadamente 3.0--3.6 Hz con stack y RViz activos.
- Interfaz web: HTTP 200 e imagen RGB disponible.
- Caida controlada del router durante 8 segundos.
- Reconexion automatica de WSL y telemetria al restaurar el router.
- Durante arranque, operacion y caida del router: `cmd_vel=0`, PWM=0 y RPM=0.

## Observaciones

- El Kinect puede reenumerarse al reiniciar el stack. Si anuncia topicos pero no
  entrega frames, un segundo reinicio de `robot-follower.service` despues de
  estabilizar USB recupera el flujo.
- GPS conectado, sin fix durante la prueba interior (`sats=0`).
- Falta validar un reinicio electrico completo del Pi y una prueba de Stadia con
  ruedas suspendidas antes de declarar el perfil productivo de campo.

## Rollback

Ejecutar `/home/josemsotov/robot_ws/zenoh/pi_rollback_to_fastdds.sh` y verificar
`/cmd_vel`, `/motor_status` y los sensores antes de continuar.
