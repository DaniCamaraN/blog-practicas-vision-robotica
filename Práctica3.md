# Marker Visual Localization: Localización Visual con AprilTags

## 1. Introducción

En este proyecto se ha desarrollado un sistema de localización visual utilizando AprilTags para estimar la posición de un robot en un entorno conocido. El sistema detecta marcadores AprilTag en imágenes de una cámara, calcula la posición y orientación del robot en coordenadas del mundo, y controla el movimiento del robot para navegar hacia los marcadores detectados.

### Objetivos principales

- Detectar AprilTags en imágenes en tiempo real.
- Estimar la posición de la cámara (y por ende del robot) en el espacio 3D.
- Transformar coordenadas entre sistemas de referencia (cámara, robot, mundo).
- Implementar un control de movimiento para buscar y acercarse a los marcadores.
- Visualizar la posición estimada en una interfaz gráfica.

---

## 2. Configuración inicial y detección de AprilTags

El primer paso consiste en configurar el detector de AprilTags y procesar las imágenes para detectar los marcadores.

### 2.1 Configuración del detector

Se utiliza la biblioteca `pyapriltags` para detectar AprilTags de la familia `tag36h11`:

```python
import pyapriltags
detector = pyapriltags.Detector(searchpath=["apriltags"], families="tag36h11")
```

Además, se carga un archivo YAML con las poses conocidas de los tags en el mundo:

```python
conf = yaml.safe_load(Path("/resources/exercises/marker_visual_loc/apriltags_poses.yaml").read_text())
tags = conf["tags"]
```

### 2.2 Detección en la imagen

La función `detect_april_tags` procesa la imagen:

1. Conversión a escala de grises:

```python
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
```

2. Detección de tags:

```python
results = detector.detect(gray)
```

3. Dibujo de bounding boxes y centros en la imagen para visualización:

```python
for r in results:
    (ptA, ptB, ptC, ptD) = r.corners
    # Dibujar líneas y círculos...
    cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
WebGUI.showImage(image)
```

Objetivo:

- Identificar la presencia y posición de los AprilTags en la imagen.
- Proporcionar feedback visual al usuario.

---

## 3. Estimación de posición de la cámara

Para cada tag detectado, se estima la transformación entre el tag y la cámara.

### 3.1 Puntos 3D del tag

Se definen los puntos 3D del tag en su sistema local:

```python
def get_tag_object_3Dpoints(tag_size=0.24):
    half = tag_size/2
    return np.array([[-half, half, 0],
                     [ half, half, 0],
                     [ half,-half, 0],
                     [-half,-half, 0]], dtype=np.float32)
```

### 3.2 Parámetros de la cámara

Se calculan la matriz de cámara y coeficientes de distorsión:

```python
def get_camera_params(image):
    h, w = image.shape[:2]
    f = w
    center = (w/2, h/2)
    camera_matrix = np.array([[f,0,center[0]],[0,f,center[1]],[0,0,1]], dtype="double")
    dist_coeffs = np.zeros((4,1))
    return camera_matrix, dist_coeffs
```

### 3.3 SolvePnP para estimar posición

Se utiliza `cv2.solvePnP` para encontrar la rotación y traslación:

```python
success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
```

---

## 4. Transformaciones de coordenadas

Se transforman las poses entre diferentes sistemas de referencia para obtener la posición del robot en el mundo.

### 4.1 Matriz de transformación cámara a robot

$$
T_{robot}^{camera} = 
\begin{pmatrix} 
1 & 0 & 0 & 0.069 \\ 
0 & 1 & 0 & -0.047 \\ 
0 & 0 & 1 & 0.107 \\ 
0 & 0 & 0 & 1 
\end{pmatrix}
$$

### 4.2 Transformación a coordenadas Gazebo

$$
R_x = 
\begin{pmatrix} 
1 & 0 & 0 & 0 \\ 
0 & 0 & 1 & 0 \\ 
0 & -1 & 0 & 0 \\ 
0 & 0 & 0 & 1 
\end{pmatrix}
$$

$$
R_z = 
\begin{pmatrix} 
0 & 1 & 0 & 0 \\
-1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 
\end{pmatrix}
$$

La transformación total es $R_z \cdot R_x \cdot T$

### 4.3 Matriz mundo a tag

$$
T_{tag}^{world} = 
\begin{pmatrix} 
\cos(\psi) & -\sin(\psi) & 0 & x \\ 
\sin(\psi) & \cos(\psi) & 0 & y \\ 
0 & 0 & 1 & 0.8 \\ 
0 & 0 & 0 & 1 
\end{pmatrix}
$$

donde $\psi$ es el yaw del tag, y $(x, y, 0.8)$ es su posición en el mundo.

### 4.4 Matriz tag a cámara

$$
T_{camera}^{tag} = 
\begin{pmatrix} R & \mathbf{t} \\ 
0 & 1 
\end{pmatrix}
$$

$$
T_{tag}^{camera} = (T_{camera}^{tag})^{-1} = 
\begin{pmatrix} R^T & -R^T \mathbf{t} \\ 
0 & 1 
\end{pmatrix}
$$

donde $R$ es la matriz de rotación obtenida de $\mathbf{rvec}$ vía Rodrigues, y $\mathbf{t}$ es el vector de traslación.

### 4.5 Cálculo de la posición de la cámara en el mundo

La posición del robot en el sistema de referencia del mundo se obtiene mediante la composición de transformaciones homogéneas:

$$
T_{robot}^{world} = 
T_{tag}^{world} \cdot 
\left( R_z \cdot R_x \cdot T_{tag}^{camera} \right) \cdot 
T_{robot}^{camera}
$$

A partir de esta matriz, la posición del robot viene dada por:

$$
(x, y) = \left( T_{robot}^{world}(1,4),\ T_{robot}^{world}(2,4) \right)
$$

y la orientación (ángulo yaw) se obtiene a partir del pitch, siendo $$\mathbb{T} = T_{robot}^{world}$$:

$$
\text{pitch} = atan2\left(-\mathbb{T}_{3,1},\ \sqrt{\mathbb{T}_{1,1}^2 + \mathbb{T}_{2,1}^2}\right)
$$

$$
\theta = atan2\left(\frac{\mathbb{T}_{2,1}}{\cos(\text{pitch})},\ \frac{\mathbb{T}_{1,1}}{\cos(\text{pitch})}\right)
$$

Finalmente, se aplica un ajuste constante al ángulo:

$$
\theta_{final} = \theta + \frac{\pi}{2}
$$

---

## 5. Control del robot y estados

El sistema implementa un control basado en estados para navegar hacia los tags.

### 5.1 Estados del robot

- **SEARCH**: Gira en el lugar para buscar tags.
- **FORWARD**: Avanza hacia el tag detectado.
- **RANDOM**: Movimiento aleatorio si no encuentra tags después de girar.

```python
state = "SEARCH"
rotation_steps = 0
max_rotation_steps = 200
random_steps = 0
```

### 5.2 Lógica de movimiento

Cuando se detecta un tag:

```python
HAL.setV(200)
HAL.setW(0)
state = "FORWARD"
```

Si no hay tag:

```python
if state == "SEARCH":
    HAL.setV(0)
    HAL.setW(0.5)
    rotation_steps += 1
    if rotation_steps > max_rotation_steps:
        state = "RANDOM"

elif state == "RANDOM":
    HAL.setV(100)
    HAL.setW(np.random.uniform(-1, 1))
    random_steps += 1
    if random_steps > 50:
        state = "SEARCH"
```

---

## 6. Odometría y actualización de posición

Cuando no hay tag visible, se actualiza la posición estimada usando odometría:

```python
if last_pose and last_odom:
    dx = odom.x - last_odom.x
    dy = odom.y - last_odom.y
    dyaw = odom.yaw - last_odom.yaw
    x = last_pose[0] + dx
    y = last_pose[1] + dy
    yaw = last_pose[2] + dyaw
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    last_pose = (x, y, yaw)
else:
    last_pose = (odom.x, odom.y, odom.yaw)
```

---

## 7. Visualización

Se muestra la imagen procesada y la posición estimada:

```python
WebGUI.showImage(image)
WebGUI.showEstimatedPose(last_pose)
```

---

## 8. Resultados

El sistema permite al robot localizar AprilTags y localizarse en el mapa.

Características:

- Estimación de posición usando mapa conocido con los tags.
- Control basado en estados.
- Actualización continua de la posición mediante odometría cuando no se visualiza ningún tag.

### Problemas encontrados

- Dependencia de la calidad de la detección de tags.
- Errores de odometría acumulados.
- Dificultad en encontrar el sistema de referencia adecuado.

---

## 9. Evolución del código / mejoras recientes

Inicialmente, la estimación de la posición del robot salía incorrecta porque no se conocían las transformaciones necesarias entre los sistemas de coordenadas: específicamente, la transformación de la cámara al robot y la conversión a coordenadas de Gazebo. Esto causaba discrepancias significativas entre la posición estimada y la real, lo que afectaba la navegación del robot.

Revisando blogs y recursos relacionados, encontré información valiosa en el blog de Javier Izquierdo (enlace: https://javizqh-urjc-practices.github.io/VR-blog/posts/P5/), donde se explican las transformaciones matriciales necesarias para alinear correctamente los sistemas de coordenadas en entornos de simulación como Gazebo. Aplicando estas correcciones, se logró una estimación precisa de la posición.

---

## Videos/Imágenes

# Demo de Localización Visual con AprilTags:
[![Marker Visual Localization Demo](https://img.youtube.com/vi/wWD-5p1L7TQ/maxresdefault.jpg)](https://youtu.be/wWD-5p1L7TQ)
