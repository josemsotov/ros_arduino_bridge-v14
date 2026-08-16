# Línea base de encoders — 2026-08-16

## Configuración evaluada

- Arduino Mega 2560, puerto COM4.
- Hall: 45 PPR.
- Optoencoder: 60 PPR.
- Opto izquierdo: D3; opto derecho: D2.
- Filtro temporal fijo: 6000 us.
- Prueba individual: 1 segundo por rueda, tres repeticiones por PWM.
- Posición cerrada: objetivo de 60 pulsos opto, cinco repeticiones.

## Reposo

Durante cinco segundos, ambas tecnologías registraron cero pulsos en las dos ruedas.

## Barrido individual hacia adelante

El error compara el opto medido contra `Hall × 60 / 45`.

| PWM | Rueda | Hall promedio | CV Hall | Opto promedio | Error absoluto medio |
|---:|:---:|---:|---:|---:|---:|
| 20 | L | 34.00 | 0.00% | 70.00 | 54.41% |
| 20 | R | 35.00 | 0.00% | 72.33 | 55.00% |
| 25 | L | 45.33 | 1.04% | 79.33 | 31.30% |
| 25 | R | 45.67 | 1.03% | 76.67 | 25.92% |
| 30 | L | 54.33 | 1.74% | 79.33 | 9.50% |
| 30 | R | 56.33 | 0.84% | 95.00 | 26.50% |
| 35 | L | 64.00 | 1.28% | 88.67 | 3.92% |
| 35 | R | 65.33 | 0.72% | 106.67 | 22.44% |
| 40 | L | 75.00 | 0.00% | 96.00 | 4.00% |
| 40 | R | 76.00 | 0.00% | 105.00 | 3.62% |

Los motores arrancaron en 3/3 intentos en todos los PWM. Hall fue muy repetible. El opto sobrecontó principalmente a baja velocidad y en el motor derecho entre PWM 30 y 35.

## Posición cerrada de 60 pulsos

| Repetición | Opto L | Opto R | Hall L | Hall R | Tiempo |
|---:|---:|---:|---:|---:|---:|
| 1 | 60 | 63 | 47 | 42 | 702 ms |
| 2 | 60 | 65 | 49 | 38 | 736 ms |
| 3 | 60 | 62 | 43 | 44 | 673 ms |
| 4 | 60 | 63 | 46 | 46 | 693 ms |
| 5 | 60 | 65 | 44 | 43 | 690 ms |

- Éxito del comando: 5/5.
- Hall esperado para una vuelta: 45 pulsos.
- Hall L: rango 43–49; error medio firmado +1.8%.
- Hall R: rango 38–46; error medio firmado -5.3%.
- Tiempo: rango 673–736 ms.

## Conclusión

La configuración actual es utilizable para movimientos lentos, pero el filtro fijo no elimina uniformemente el ruido en todo el rango. Toda mejora posterior debe compararse contra estas métricas y conservar una prueba de regresión con el mismo protocolo.

## Mejora 1 — instrumentación del filtro

Se añadieron contadores independientes de flancos recibidos, aceptados y rechazados. El filtro permaneció fijo en 6000 us.

- Reposo: 0 pulsos.
- Posición cerrada: 5/5.
- Hall L: rango 42–49; error medio firmado +1.8%.
- Hall R: rango 39–46; error medio firmado -6.2%.
- Tiempo: rango 686–732 ms.
- Flancos observados durante movimiento: aproximadamente 250–645 por segundo.
- El canal derecho recibió consistentemente más interferencia que el izquierdo.

Decisión: integrada. Aporta diagnóstico y no produjo una regresión material.

## Mejora 2 — filtro adaptativo independiente

El filtro se calcula desde el intervalo Hall de cada rueda, disminuye inmediatamente al acelerar y aumenta gradualmente al desacelerar. El arranque usa una ventana inicial basada en PWM.

| PWM | Error base L | Error adaptativo L | Error base R | Error adaptativo R |
|---:|---:|---:|---:|---:|
| 20 | 54.41% | 7.69% | 55.00% | 1.67% |
| 25 | 31.30% | 4.44% | 25.92% | 3.68% |
| 30 | 9.50% | 1.65% | 26.50% | 0.15% |
| 35 | 3.92% | 4.80% | 22.44% | 1.81% |
| 40 | 4.00% | 2.33% | 3.62% | 0.33% |

- Todos los errores medios quedaron por debajo de 8%.
- Peor repetición individual: 12.5%.
- Reposo: 0 pulsos.
- Posición cerrada: 5/5.
- Posición adaptativa Hall L: rango 43–48; error medio firmado +4.4%.
- Posición adaptativa Hall R: rango 42–47; error medio firmado +0.9%.
- Tiempo adaptativo: rango 643–724 ms.

Decisión: integrada. Cumple error medio menor a 10%, peor caso menor a 15%, cero pulsos en reposo y 5/5 posiciones cerradas.

## Mejora 3 — supervisor Hall–opto en diagnóstico

Se añadió un clasificador por ventanas con tolerancia de ±20% y confirmación de tres ventanas anómalas antes de declarar `LOW` o `HIGH`. En esta etapa no interviene en los motores.

- Autoprueba interna `IDLE`, `OK`, `LOW`, `HIGH`: PASS.
- Clasificación física: 30/30 pruebas correctas.
- Rueda activa: `OK`; rueda detenida: `IDLE`.
- Sin falsos avisos.
- Error medio máximo en la regresión: 6.97%.
- Posición cerrada: 5/5.
- Hall L: rango 43–48; error medio firmado +0.9%.
- Hall R: rango 45–47; error medio firmado +2.2%.
- Tiempo: rango 673–722 ms.
- RAM libre después de integrar el supervisor: 1545 bytes.

Decisión: integrada en modo diagnóstico. La activación de parada o fallback queda pendiente de una prueba controlada de fallas reales.

## Rendimiento de posicionamiento con mejoras 1–3

Prueba escalonada con tres repeticiones por objetivo y ambos motores simultáneos.

| Objetivo opto | Vueltas | Éxito | Error abs. L | Error abs. R | Desfase Hall medio | Tiempo medio |
|---:|---:|---:|---:|---:|---:|---:|
| 15 | 0.25 | 3/3 | 8.15% | 5.19% | 0.3 pulsos | 236.7 ms |
| 30 | 0.50 | 3/3 | 6.67% | 3.70% | 1.0 pulso | 406.0 ms |
| 60 | 1.00 | 3/3 | 1.48% | 3.70% | 1.0 pulso | 705.3 ms |
| 120 | 2.00 | 3/3 | 1.85% | 1.48% | 1.0 pulso | 1273.3 ms |

- Resultado global: 12/12 movimientos completados sin timeout.
- Sobrepaso opto máximo observado: 3 pulsos en el objetivo de 60 y 2 pulsos en el de 120.
- El mayor porcentaje relativo en 15 pulsos corresponde a una diferencia absoluta pequeña y a la resolución de 45 PPR del Hall.
- El control actual muestra mejor precisión relativa desde una vuelta completa.
