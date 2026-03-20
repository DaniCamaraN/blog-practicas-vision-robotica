# 3D Reconstruction: Reconstrucción 3D con Visión Estéreo

## 1. Introducción

En este proyecto se ha desarrollado un sistema de reconstrucción 3D a partir de un par de cámaras estéreo utilizando técnicas de visión por computador.

### Objetivos principales

- Detectar puntos de interés en ambas imágenes.
- Encontrar correspondencias entre imágenes (matching).
- Aplicar geometría epipolar para restringir la búsqueda.
- Triangular puntos en el espacio 3D.
- Generar una nube de puntos coloreada.

---

## 2. Fase 1 — Detección de puntos de interés

El primer paso consiste en detectar puntos relevantes en la imagen izquierda que posteriormente se intentarán encontrar en la imagen derecha.

### 2.1 Procesado de imagen

Se aplicaron varias etapas:

1. Conversión a escala de grises:

```python
grayL = cv2.cvtColor(imageL, cv2.COLOR_BGR2GRAY)
```

2. Mejora de contraste con CLAHE:

```python
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
grayL = clahe.apply(grayL)
```

3. Suavizado Gaussiano:

```python
grayL = cv2.GaussianBlur(grayL, (5,5), 0)
```

4. Detección de bordes con Canny:

```python
edgesL = cv2.Canny(grayL, 50, 150)
```

### 2.2 Selección de puntos

Se seleccionan píxeles de borde como puntos de interés:

```python
points = np.argwhere(edges > 0)
```

Y se elige un subconjunto aleatorio:

```python
N = 500
points = points[np.random.choice(len(points), N, replace=False)]
```

Objetivo:

- Reducir carga computacional.
- Mantener suficiente información estructural.

---

## 3. Fase 2 — Retroproyección (Projection Rays)

Para cada punto en la imagen izquierda, se calcula un rayo 3D que representa todas las posibles posiciones del punto en el espacio 3D.

```python
ray_L = projection_ray('left', (y,x))
```

Este rayo se define como:

- Punto origen → posición de la cámara.
- Dirección → backprojection del píxel.

> Concepto clave: cada píxel en la imagen corresponde a un rayo en el espacio 3D, no a un punto único.

---

## 4. Fase 3 — Geometría epipolar

Buscar el punto correspondiente en toda la imagen derecha sería muy costoso. La restricción epipolar acota la búsqueda a una línea.

```python
mask = epipolar('right', ray_L, grayR.shape)
```

Esto genera una máscara donde solo se buscarán coincidencias.

### Ventajas

- Reduce drásticamente el espacio de búsqueda.
- Mejora precisión del matching.
- Reduce falsos positivos.

---

## 5. Fase 4 — Búsqueda de puntos homólogos

Una vez definida la línea epipolar, se busca el punto correspondiente usando template matching.

```python
match, conf = homologous((y,x), grayL, grayR, mask)
```

Se utiliza:

```python
cv2.matchTemplate(..., cv2.TM_CCOEFF_NORMED)
```

### Filtrado por confianza

```python
if conf < 0.9:
    continue
```

### Filtrado adicional

```python
if abs(x - xR) < 5:
    continue
```

Esto elimina correspondencias poco fiables o ambiguas.

---

## 6. Fase 5 — Triangulación 3D

Una vez tenemos los dos rayos (izquierda y derecha), se calcula su intersección:

```python
A = np.vstack((d1[:3], -d2[:3])).T
b = p2[:3] - p1[:3]

t1, t2 = lstsq(A, b)

P1 = p1[:3] + t1*d1[:3]
P2 = p2[:3] + t2*d2[:3]

point3D = (P1 + P2) / 2
```

Idea clave: Debido a errores, los rayos no se cruzan exactamente, por lo que se toma el punto medio.

---

## 7. Fase 6 — Filtrado por consistencia

Se mide la distancia entre los dos rayos:

```python
dist = np.linalg.norm(P1 - P2)
```

Y se filtra:

```python
if dist < threshold:
    ...
```

Esto elimina reconstrucciones erróneas.

---

## 8. Fase 7 — Color de los puntos

Cada punto 3D se colorea usando el color original de la imagen:

```python
color = imageL[y, x]
points3D.append([x, y, z, R, G, B])
```

---

## 9. Fase 8 — Visualización

Finalmente se muestra la nube de puntos:

```python
gui.ShowNewPoints(points3D)
```

También se incluyen las posiciones de las cámaras para referencia:

- Cámara izquierda → azul
- Cámara derecha → verde

---

## 10. Resultados

El sistema genera una nube de puntos 3D coloreada en tiempo real.

Características:

- Representación estructural del entorno.
- Buen detalle en bordes.
- Dependencia de la calidad del matching.

### Problemas encontrados

- Ruido en la detección de bordes.
- Falsas correspondencias en zonas homogéneas.
- Sensibilidad al parámetro de confianza.
- Puntos mal reconstruidos por rayos casi paralelos.

## Video

[![3D Reconstruction Demo](https://img.youtube.com/vi/_J_dqgsMbxw/maxresdefault.jpg)](https://youtu.be/_J_dqgsMbxw)