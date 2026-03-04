# Follow Line: Control PID para Seguimiento de Línea

## Introducción

En este proyecto se ha desarrollado un sistema de control para un robot móvil capaz de seguir una línea roja en un circuito utilizando visión artificial y un controlador PID.

El objetivo principal ha sido:

- Detectar correctamente la línea en tiempo real.
- Calcular un error robusto (posición + orientación).
- Diseñar un controlador PID estable.
- Adaptar dinámicamente la velocidad según la curvatura.
- Optimizar tiempos por vuelta.

---

## Fase 1 — Detección básica de la línea

### Primera versión: detección con un solo punto

Inicialmente, la línea se detectaba mediante:

1. Conversión de la imagen a espacio **HSV**.
2. Filtrado por color rojo (doble rango en HSV).
3. Cálculo del centro horizontal de la línea en una zona inferior de la imagen.

El error se calculaba simplemente como:

$$\text{error = centro\_imagen - x\_linea}$$


#### ❌ Problema detectado

Aunque el robot seguía rectas correctamente, en curvas:

- Reaccionaba tarde.
- Oscilaba.
- Tomaba trayectorias poco suaves.
- Perdía la línea en curvas cerradas.

Esto ocurría porque solo se tenía **información posicional**, pero no información sobre la **orientación** de la línea.

---

## Fase 2 — Mejora: detección con dos puntos

Para solucionar el problema anterior, se decidió calcular **dos puntos** de la línea:

- **Punto inferior** → referencia principal de posición.
- **Punto superior** → permite estimar inclinación.

Esto permite calcular:


$$\text{error_position = center\_image - x\_bottom}$$
$$\text{error_angle = x\_bottom - x\_top}$$

Y el error total pasa a ser:

$$\text{error = weight\_pos * error\_position + weight\_ang * error\_angle}$$

Teniendo que ajustar estos dos pesos, dando más o menos importancia a la posición o al ángulo.

### Ventajas

- El robot **anticipa curvas**.
- Movimiento más **suave**.
- Reducción de **oscilaciones**.
- Mejor estabilidad en trazados rápidos.

### Decisión importante

Se introdujeron pesos:

- `weight_pos = 1`
- `weight_ang = 0.92`

Esto permitió ajustar cuánto influye cada componente del error.

Tras pruebas empíricas:

- Si el peso angular era muy alto → comportamiento nervioso.
- Si era muy bajo → pérdida de anticipación en curvas.

---

## Fase 3 — Diseño del PID de giro

Una vez definido un error más robusto, se implementó el controlador:


$$\text{w = Kp * error + Kd * derivative + Ki * integral}$$


### Parámetros finales:

- `Kp = 0.021`
- `Kd = 0.017`
- `Ki = 0.000001`

### Proceso de ajuste

#### 1. Ajuste de Kp

- Valor bajo → giro lento.
- Valor alto → oscilaciones fuertes.

Se buscó un punto donde:

- Corrigiera rápido.
- Sin vibraciones excesivas.

#### 2. Ajuste de Kd

Se añadió para:

- Reducir sobreoscilación.
- Mejorar estabilidad en curvas rápidas.

Al aumentar Kd:

- Movimiento más estable.
- Mejor trazado en curvas enlazadas.

#### 3. Ajuste de Ki

Se añadió para:

- Corregir pequeños errores persistentes.

Se dejó muy pequeño porque:

- Valores altos generaban inestabilidad.
- El sistema no necesitaba mucha corrección acumulativa.

---

## Fase 4 — Control dinámico de velocidad

Una mejora clave fue hacer que la velocidad dependiera del **ángulo**.

$$\text{v = V\_max - Kp\_v * abs(error\_angle) - Kd\_v * abs(derivative)}$$


### Parámetros:

- `V_max = 16`
- `V_min = 3`
- `Kp_v = 0.03`
- `Kd_v = 0.1`

### Objetivo

- Ir rápido en rectas.
- Reducir velocidad en curvas.
- Mantener estabilidad sin perder tiempo.

---

## Resultados

- **Mejor vuelta inicial:** 166.53 s
- **Tras optimización (20% RF):** 133.23 s

La mejora se debió principalmente a:

- Uso del error angular.
- Ajuste fino del PID.
- Control adaptativo de velocidad.

---

## Manejo de pérdida de línea

Se añadió un sistema de recuperación:

```python
if lost_counter < MAX_LOST:
    HAL.setW(2 * last_seen_direction)
else:
    HAL.setW(4 * last_seen_direction)
```
Esto permite:

- Intentar recuperar suavemente.

- Si no se encuentra la línea, buscar más agresivamente.

- Mantener memoria de la última dirección vista.

## Conclusiones Técnicas
### Lo más importante aprendido:

- Solo error posicional no es suficiente.
- La orientación mejora drásticamente el control.
- El PID necesita ajuste empírico iterativo.
- La velocidad dinámica reduce tiempos significativamente.