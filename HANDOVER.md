# Handover — Smart Trolley V14

Actualizado: 2026-08-20


## Actualizacion 2026-08-20 - plataforma de movimiento y odometria

- Workspace local activo migrado a `D:\1-EXTERNAL\PROYECTOS JMS 2025\SMART-TROLLEY-JUN-2026\MOTOR-INTERFACE-V14`.
- Raspberry Pi accesible en `josemsotov@192.168.40.74`.
- Zenoh persistente y servicios `smart-trolley-zenoh-router`, `robot-follower` y `robot-operator-web` activos.
- Diametro fisico real: `wheel_dia=0.27 m`; Pi recompilado y parametro activo verificado.
- Optoencoders declarados a `ppr=60`: D3 izquierdo y D2 derecho. El comando `e` publica optos; Hall permanece para RPM/diagnostico.
- IMU activa con fusion inicial yaw/gyro Z. GPS comunica, pero la ultima verificacion seguia sin fix ni satelites.
- Prueba terrestre util: inicio `L=1794 R=2225`, final `L=1988 R=2345`, distancia fisica `30.3 cm`; deltas `L=194 R=120`.
- Hay asimetria significativa entre encoders. La calibracion de odometria no esta cerrada.
- El Pi tuvo perdidas temporales de SSH, sin reinicio ni bajo voltaje (`get_throttled=0x0`); revisar Wi-Fi.
- Estado seguro al cierre: servicios activos, `/cmd_vel=0`, robot en `PAUSE`.

### Siguiente paso recomendado

1. Repetir dos recorridos terrestres rectos y medidos con Stadia.
2. Registrar conteos iniciales/finales por rueda y distancia fisica.
3. Evaluar factores de escala separados izquierda/derecha.
4. Conservar el diametro fisico `0.27 m`; no usarlo para ocultar asimetrias de encoder.
5. Validar avance, retroceso y parada antes de navegacion autonoma.
## Estado de cierre

- Raspberry Pi 5 de 8 GB: `josemsotov@192.168.40.73`.
- Interfaz: `http://192.168.40.73:8080`.
- Servicios activos:
  - `robot-follower.service`
  - `robot-operator-web.service`
- Estado seguro verificado:
  - modo `STADIA`
  - follower deshabilitado
  - `/cmd_vel = 0 / 0`
  - PWM `0/0`
  - RPM `0/0`
- No se realizó ninguna prueba con movimiento real.
- Mantener el robot suspendido para las próximas pruebas.

## Safety net vigente

- Stadia es el modo predeterminado y toma control al conectarse por Bluetooth.
- El mando exige palancas centradas antes de armarse.
- Solo `/follower/authorized_enable` puede habilitar intencionalmente FOLLOWER.
- Stadia, OFF, desconexión y STOP cancelan autorizaciones anteriores.
- El follower exige una sesión facial válida para arrancar.
- Watchdog Arduino: `cmd_timeout=0.5`.
- Paro reproducible:
  `.codex_runtime_fix/pi8_safety_net/emergency_stadia_stop.sh`.

## Cambios desplegados el 2026-07-24

En `follower_node.py` y `follower_params.yaml`:

- Agrupación de puntos LiDAR contiguos y selección por alineación visual,
  continuidad de distancia y tamaño.
- Telemetría de distancia/ángulo crudos, cantidad de clústeres y estado.
- Procesamiento body–LiDAR disponible en `FACE_STATIC_DRY_RUN`, siempre con salida cero.
- Sin body track fresco:
  - estado `no_person_track`
  - STOP
  - no se crea un objetivo nuevo.
- Un objetivo inicial exige rostro visible, sesión válida e identidad verificada.
- Cada enrolamiento reinicia completamente la referencia LiDAR heredada.
- Se conserva la última referencia durante pérdidas visuales breves.
- Compuerta de distancia facial:
  - escala inicial `face_distance_scale_m=0.185`
  - margen `lidar_face_distance_gate_m=0.45`
  - timeout `1.50 s`
  - estado de rechazo `face_distance_mismatch`.
- Compensación de montaje cámara–LiDAR:
  - `lidar_camera_yaw_deg=90.0`
  - ángulos normalizados a `[-π, π]`.
- Política final de continuidad:
  - cambios mayores de `0.25 m` durante la misma sesión se rechazan
  - estado `distance_discontinuity`
  - STOP y conservación de la última distancia válida
  - ya no se acepta otro objeto después de varios barridos.

## Resultados observados

### Aprobado

- Rostro frontal:
  - `face_x=0.526–0.532`
  - identidad verificada.
- Body track aproximadamente `0.94`.
- Después de compensar `90°`, adquisición corporal estable:
  - distancia `1.23–1.29 m`
  - clúster `16–22` puntos
  - ángulo aproximado `0.05 rad`.
- En todas las muestras:
  - `/cmd_vel=0`
  - PWM/RPM en cero.

### Falló de forma segura

- En la prueba lateral, al salir parcialmente del cuadro el selector migró desde
  `≈1.27 m` hacia un objeto de `≈0.72–0.76 m`.
- La sesión facial y el body track se perdieron temporalmente.
- No hubo movimiento porque el modo era dry-run.
- La causa fue la política anterior que aceptaba una discontinuidad después de tres
  barridos consistentes.
- Esa política fue eliminada y sustituida por rechazo estricto de cambios `>0.25 m`.

### Pendiente de validar

La nueva política `distance_discontinuity` fue desplegada, compilada y el servicio quedó
activo, pero todavía no se repitió la adquisición central ni la prueba lateral después
de este último cambio.

## Reanudación recomendada

1. Confirmar robot suspendido y paro físico accesible.
2. Abrir `http://192.168.40.73:8080`.
3. Colocarse a aproximadamente `1.3 m`, torso centrado y rostro mirando al lente.
4. Iniciar `FACE_STATIC_ENROLL`.
5. Aprobar adquisición central solo si:
   - identidad y sesión válidas
   - body track fresco
   - LiDAR `≈1.2–1.4 m`
   - clúster corporal consistente
   - `/cmd_vel`, PWM y RPM en cero.
6. Repetir desplazamiento lateral en dry-run.
7. Confirmar que un objeto a `≈0.72 m` produzca `distance_discontinuity` y que la
   referencia corporal no cambie.
8. Regresar al centro y confirmar recuperación a `≈1.2–1.4 m`.
9. Solo después evaluar una prueba muy limitada con ruedas suspendidas.
10. No apoyar las ruedas hasta validar STOP, pérdida de persona y takeover de Stadia.

## Fuente activa respaldada

- `.codex_runtime_fix/pi8_safety_net/current/follower_node.py`
- `.codex_runtime_fix/pi8_safety_net/current/follower_params.yaml`
- `.codex_runtime_fix/pi8_safety_net/current/stadia_node.py`
- `.codex_runtime_fix/pi8_safety_net/current/arduino_node.py`
- `.codex_runtime_fix/pi8_safety_net/web_current/`

Las capturas de cámara y cachés Python permanecen excluidas de GitHub.

## Comandos útiles

```text
ssh josemsotov@192.168.40.73
systemctl --user status robot-follower.service robot-operator-web.service
```

## 2026-08-21 - Perfil ampliado Hall/opto (banco suspendido)

- `MAX_PWM_VALUE` operativo permanece en 40 para ROS2/Stadia.
- El comando diagnostico `q <L|R> <pwm>` tiene limite independiente `DIAGNOSTIC_MAX_PWM = 80`.
- Firmware compilado (59990 bytes), respaldado en Pi como `/home/josemsotov/robot_backups/pre_20260821_diag_pwm80.hex`, cargado y verificado con avrdude.
- Matriz: PWM 10,15,20,25,30,35,40,50,60,70,80; 3 repeticiones por rueda; Hall 45 PPR contra opto 60 PPR.
- Zona mas consistente: PWM 25-40 (aprox. -3.4% a +5.4% de error medio, excepto dispersion puntual izquierda a 40).
- PWM 10-15: sobreconteo fuerte; PWM 50-80: subconteo creciente, alrededor de -20% a -23% desde PWM 60.
- Informes: `encoder_calibration_reports/encoder_cross_extended_20260821.{json,csv}`.
- Al terminar: `robot-follower.service` activo, `/motor_status` Lpwm=0 Rpwm=0 Lrpm=0 Rrpm=0.
## 2026-08-21 - Filtro opto adaptativo V3 validado

- Mejora 1: limites adaptativos ampliados de L=5500/R=6000..15000 us a L=2500/R=2500..40000 us.
- Mejora 2 evaluada: ganancia izquierda por bandas; corrigio extremos pero sobrefiltro PWM 15-20.
- Mejora 3 activa: ganancia izquierda por intervalo Hall: >=50000 us:650 permille; >=27000:500; >=12000:450; >=8000:375; menor:350. Derecha conserva 520 permille.
- Comparacion misma matriz (PWM 10..80, 3 repeticiones/rueda): baseline MAE 18.23%, V1 6.00%, V2 4.13%, V3 3.14%.
- Peor muestra: baseline 118.75%; V3 10.71%.
- Informes JSON/CSV en `encoder_calibration_reports/` para baseline, adaptive_bounds_v1, left_piecewise_v2 y left_piecewise_v3.
- `MAX_PWM_VALUE` operativo sigue en 40; `DIAGNOSTIC_MAX_PWM` sigue en 80.
- Estado final: follower y Zenoh activos; motor_status Lpwm=0 Rpwm=0 Lrpm=0 Rrpm=0.
## 2026-08-21 - Base de lazo cerrado: fusion Hall/opto

- Firmware `e` extendido: `e <optoL> <optoR> <hallL> <hallR>`; conteos acumulados tomados atomicamente.
- ROS2 incorpora `WheelEncoderFusion` con ventana de 10 muestras (~0.5 s): <=5% usa OPTO; 5-12% BLEND; >12% fallback HALL; PWM=0 fuerza STOP y delta cero.
- Nuevo topico `/encoder_fusion/status`; `/encoder_counts` incluye fusion y los cuatro conteos crudos.
- Covarianza de `/odom` aumenta automaticamente cuando baja la confianza.
- Compatibilidad preservada con trama antigua `e <L> <R>`.
- Pruebas unitarias directas: OPTO, BLEND, HALL fallback y STOP aprobadas. Firmware 60184 bytes, verificado por avrdude.
- Respaldo firmware: `/home/josemsotov/robot_backups/pre_20260821_dual_encoder_fusion.hex`.
- Respaldo nodo Pi: `arduino_node.py.pre_encoder_fusion_20260821.bak`.
- Validacion estatica: follower activo; fusion L/R=STOP conf=1.00; odom linear/angular=0; motor PWM/RPM=0.
- Observacion: OL acumulo flancos crudos con PWM=0; fueron rechazados completamente por la fusion/guardia de reposo.
- Pendiente antes de ajustar PID: prueba dinamica controlada para observar transiciones OPTO/BLEND/HALL y confirmar si el robot esta suspendido o en suelo.
### Prueba dinamica suspendida de fusion

- Escalones comandados: 0.08, 0.15 y 0.25 m/s, 3 s cada uno, con parada entre escalones.
- El puente se ejecuto aislado temporalmente para evitar ceros del `cmd_vel_mux`; el servicio completo fue restaurado automaticamente.
- Se observaron correctamente los estados OPTO, BLEND y HALL. Ante discrepancia del opto izquierdo, la odometria uso Hall como fallback.
- Resultado acumulado fusionado de la corrida: L=239.000, R=234.833 pulsos equivalentes; conteos crudos finales OL=601 OR=262 HL=199 HR=204 (incluyen acumulados previos al inicio).
- La seleccion cambia durante transitorios; para el primer ajuste de velocidad usar PI (D=0) y agregar histeresis antes de habilitar derivada.
- Estado posterior: follower activo, PWM/RPM=0 y fusion STOP conf=1.00 en ambas ruedas.
## 2026-08-21 - PI de velocidad V1 (prueba suspendida)

- Fusion ROS2: histeresis de 3 ventanas antes de cambiar OPTO/BLEND/HALL; prueba unitaria aprobada.
- Firmware: PI independiente por rueda aplicado como correccion sobre FF; Hall alimenta el lazo interno y odometria Hall/opto fusionada queda para el lazo exterior de posicion.
- Seguridad: PI apagado al arrancar; `k on`, `k off`, `k <Kp> <Ki>`; correccion limitada a +/-6 PWM; integral limitada a +/-30; D=0.
- Telemetria T agrega Ltrpm/Rtrpm, Lpi/Rpi y PI.
- A/B a 0.15 m/s (objetivo 10.6 RPM): PI off = L/R 31.2/31.2 RPM, PWM 15.2/15.2; Kp=0.15 = 25.5/26.2 RPM, PWM 12.9/12.8.
- Barrido Ki=0: Kp=0.25 -> 18.9/19.8 RPM, PWM 12.3/12.1; Kp=0.40 -> 20.0/20.9; Kp=0.60 -> 18.5/20.6. Se selecciona Kp=0.25.
- Candidata persistente: Kp=0.25, Ki=0, Kd=0; PI permanece desactivado hasta prueba con carga en suelo.
- Firmware final 62344 bytes, RAM 6760/8192 (82%, libres 1432), escrito y verificado por avrdude.
- Respaldo previo en Pi: `/home/josemsotov/robot_backups/pre_20260821_velocity_pi_v1.hex`.
- Scripts reproducibles: `run_velocity_pi_test.sh`, `run_velocity_pi_kp_sweep.sh`; logs en `.diagnostics/velocity_pi_*_20260821.log`.
- Estado final: follower activo, PWM/RPM=0, fusion STOP conf=1.00.

## 2026-08-21 - PI de velocidad V1 en suelo

- Robot probado en suelo con dos ruedas caster; sus tirones mecanicos se excluyen del criterio de ajuste.
- Stadia confirmo movimiento con PI apagado: en reversa PWM 12 no sostuvo RPM; ambas ruedas mostraron movimiento desde aproximadamente PWM 17. En avance el primer punto simultaneo capturado fue PWM 28-29 por falta de una rampa suficientemente fina.
- Rampa lenta de avance con PI apagado: PWM 10 produjo pulsos Hall esporadicos, sin regulacion continua.
- PI activado en runtime con k 0.25 0.0 y k on.
- Prueba PI en suelo: objetivo aproximado L=7.7/R=7.3 RPM; Hall alterno entre 0 y 11-13 RPM; correcciones tipicas Lpi/Rpi de 0 a aproximadamente -1.4 PWM; salida conmutando entre PWM 0/10/11.
- Diagnostico: el PI funciona, pero el estimador Hall actual usa ventanas de 100 ms y cuantiza demasiado a baja velocidad. No ajustar Kp/Ki sobre esta medicion.
- Proximo paso: estimador Hall hibrido (periodo entre pulsos a baja velocidad, conteo por ventana a velocidad media/alta), suavizado ligero y timeout explicito a cero; repetir la misma prueba A/B antes de ajustar Ki.
- Cierre de sesion: comando cero enviado, k off, Lpwm=0 Rpwm=0 Lrpm=0 Rrpm=0 y robot-follower.service activo.

## 2026-08-22 - Hall hibrido V2 y PI de baja velocidad V4

- Firmware instalado y verificado por `avrdude`: 63524 bytes flash; 6784/8192 bytes RAM (82%, 1408 libres).
- Estimador Hall hibrido: conteo por ventana con >=3 pulsos y periodo entre flancos a baja velocidad; suavizado y timeout adaptativo; flancos menores de 6000 us excluidos del estimador de velocidad.
- La temporizacion de velocidad Hall queda separada del intervalo usado por el filtro opto adaptativo, evitando picos falsos de 691-1120 RPM observados en V1.
- PWM subminimo: modulacion por densidad de pulsos con quantum determinista de 50 ms; el PI conserva demanda fraccional antes de convertirla a pulsos 0/10.
- Autoridad PI asimetrica: correccion positiva limitada a +6 PWM y negativa a -10 PWM. PI permanece apagado al arrancar y al finalizar pruebas.
- Barrido suspendido a 0.10 m/s, objetivo 7.1 RPM, Ki=0: Kp 0.25 = 14.82/15.45 RPM; Kp 0.50 = 12.12/13.04; Kp 0.75 = 11.73/12.35. Kp 1.0 y 1.5 no mejoraron porque las ruedas continuaron girando por inercia aun con duty cercano a cero.
- Candidato conservador para la siguiente prueba con carga: Kp=0.75, Ki=0. No dejarlo persistente ni activado hasta validar en suelo con rampa corta desde 0.06 m/s.
- Logs reproducibles: `.diagnostics/low_speed_kp_0.25.log`, `0.50`, `0.75`, `low_speed_v4_kp_0.75.log`, `low_speed_v4_highkp_1.00.log` y `1.50.log`.
- Estado final confirmado: `robot-follower.service` activo; `lin=0`, PWM L/R=0/0, RPM L/R=0/0.
- Proximo paso: colocar robot en suelo, despejar trayectoria y ejecutar escalon/rampa limitada a 0.06 m/s; comparar Kp 0.50 y 0.75 antes de introducir Ki.