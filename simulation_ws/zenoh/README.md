# Enlace ROS 2 Zenoh para el robot

Perfil inicial de validacion. El router escucha TCP 7447 en el Raspberry Pi.
El Kinect puede ejecutarse temporalmente con `rmw_zenoh_cpp` mientras el resto
del stack permanece en Fast DDS. WSL/RViz se conecta como cliente TCP al Pi.

No activar permanentemente hasta completar las pruebas de latencia, perdida,
reconexion y parada segura. Para rollback, detener los procesos de prueba y
reiniciar `robot-follower.service`.

## Despliegue permanente

- `systemd/smart-trolley-zenoh-router.service`: router TCP con reinicio automatico.
- `systemd/robot-follower-zenoh.conf`: entorno y dependencia del stack principal.
- `systemd/robot-operator-web-zenoh.conf`: entorno y dependencia de la interfaz web.

Rollback: deshabilitar el router, retirar los dos drop-ins, ejecutar
`systemctl --user daemon-reload` y reiniciar ambos servicios. El firmware del
Arduino no cambia.

El script `pi_rollback_to_fastdds.sh` automatiza exactamente ese rollback.
