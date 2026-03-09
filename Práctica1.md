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
```
error = centro_imagen - x_linea
```

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

```
error_position = center_image - x_bottom
error_angle = x_bottom - x_top
```
Y el error total pasa a ser:
```
error = weight_pos * error_position + weight_ang * error_angle
```
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

```
w = Kp * error + Kd * derivative + Ki * integral
```

### Parámetros finales (primera versión):

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
```
v = V_max - Kp_v * abs(error_angle) - Kd_v * abs(derivative)
```

### Parámetros (primera versión):

- `V_max = 16`
- `V_min = 3`
- `Kp_v = 0.03`
- `Kd_v = 0.1`

### Objetivo

- Ir rápido en rectas.
- Reducir velocidad en curvas.
- Mantener estabilidad sin perder tiempo.

---

## Resultados iniciales

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
    HAL.setW(2 * search_direction)
    HAL.setV(4)
else:
    HAL.setW(4 * search_direction)
    HAL.setV(-2)
```
Esto permite:

- Intentar recuperar suavemente.

- Si no se encuentra la línea, buscar más agresivamente.

- Mantener memoria de la última dirección vista.

## Evolución del código: Adaptación a nuevos circuitos

Tras completar la primera versión del controlador, se probó el sistema en distintos circuitos como **Montmeló** y **Nurburgring**.

Aunque el comportamiento en el circuito simple inicial era bueno, los resultados en estos nuevos circuitos no fueron satisfactorios.

### Problemas detectados:

Durante las pruebas aparecieron varios problemas:

- El controlador dependía demasiado del error angular, lo que generaba correcciones demasiado agresivas en curvas amplias.

- La detección con dos puntos era más sensible y oscilaba mucho en rectas.

- El PID estaba muy ajustado al circuito inicial, por lo que no generalizaba bien.

- El robot podía oscilar en curvas largas.

Debido a esto se decidió rediseñar parcialmente el sistema para hacerlo más **robusto** y **generalizable**.

### Nueva estrategia de detección

En la nueva versión se simplificó la detección utilizando un **único** punto inferior de la línea, al principio calculado a partir del centro de masa de los píxeles detectados, pero posteriormente se detectó unicamente con la media de los 20 primeros puntos detectados que se encontraban mas abajo de la línea detectada. Esto se hizo para simplificar el error para el posterior PID.

### Suavizado del error

Durante las pruebas también se observó que el centro detectado podía variar bruscamente entre frames. Para reducir estas variaciones se añadió un filtro de suavizado:

```python

cx_smooth = 0.7 * previous_cx + 0.3 * cx_near
error = center_image - cx_smooth

```

Gracias a esto:

- Se redujo el ruido visual.

- El control fue más estable.

- Se evitó giros bruscos.

### Nuevo ajuste del PID

Debido al cambio en la forma de calcular el error, fue necesario reajustar completamente el PID.

`Kp = 0.005`
`Kd = 0.002`
`Ki = 0.0001`

Comparado con la versión anterior:

- Se redujo mucho Kp para evitar sobrecorrecciones.

- Se redujo Kd para suavizar la respuesta.

- Se aumentó Ki ligeramente para corregir errores acumulados.

### Control del término integral

Se añadió además un mecanismo para evitar que el término integral crezca demasiado (integral windup).

El sistema solo acumula integral cuando el error es significativo:

```python
if abs(error) > 25:
    integral += error
else:
    integral *= 0.9
```

### Limitación de la velocidad angular

Se añadiero límites al término integral y a la velocidad angular

```python
max_integral = 3000
integral = max(min(integral, max_integral), -max_integral)

max_w = 3
w = max(min(w, max_w), -max_w) 
```

### Resultados de la nueva versión

Tras aplicar estas mejoras:

- El sistema se comporta de forma estable en diferentes circuitos.

- Se reduce el número de pérdidas de línea.

- El control es más suave.

La mejor vuelta obtenida fue:

55 segundos (tiempo simulado)

El tiempo se midió en simulación debido a que el Real Time Factor (RF) era inferior al 20%, lo que impedía usar tiempo real fiable.

### Conclusión final del proyecto

A lo largo del desarrollo se aprendieron varias lecciones importantes:

- Un sistema que funciona bien en un circuito no necesariamente generaliza a otros.

- A veces simplificar la detección mejora la robustez.

- Limitar el término integral evitó inestabilidades.

El resultado final es un controlador más robusto, estable y adaptable a distintos circuitos.

## Video

[![Demo PID Follow Line](https://img.youtube.com/vi/QQivoc8XLO0/maxresdefault.jpg)](https://youtu.be/QQivoc8XLO0)