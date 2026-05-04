# Follow Line End-to-End

## 1. Introducción

En este proyecto se ha desarrollado un sistema end-to-end utilizando aprendizaje profundo. El sistema utiliza un modelo de red neuronal convolucional (CNN) basado en PilotNet para predecir comandos de control (velocidad lineal `v` y velocidad angular `w`) directamente desde imágenes de la cámara del robot. El entrenamiento se realiza con un dataset combinado descargado de Hugging Face, y el modelo se exporta a formato ONNX para su uso en inferencia.

### Objetivos principales

- Descargar y preparar un dataset de imágenes etiquetadas para seguimiento de línea.
- Implementar un modelo PilotNet para predicción end-to-end.
- Entrenar el modelo con PyTorch.
- Exportar el modelo entrenado a ONNX para despliegue.
- Evaluar el rendimiento del modelo en tareas de navegación autónoma.

---

## 2. Descarga del Dataset

El primer paso es obtener el dataset necesario para el entrenamiento. Se utiliza la biblioteca `huggingface_hub` para descargar un dataset público.

### 2.1 Función de descarga

```python
from huggingface_hub import snapshot_download

def download_dataset_follow_line():
    snapshot_download(
        repo_id="JdeRobot/e2e-visual-control-combined-dataset",
        repo_type="dataset",
        token="TU_TOKEN_HF",
        local_dir="./dataset_combined"
    )
```

Esta función descarga el dataset en el directorio local `./dataset_combined`. Es necesario proporcionar un token de Hugging Face para acceder a repositorios privados.

Objetivo:

- Obtener datos de entrenamiento.

---

## 3. Limpieza del Dataset

Antes de usar el dataset, se limpia para eliminar entradas inválidas, como imágenes faltantes.

### 3.1 Función de limpieza

```python
import pandas as pd
import os

def clean_dataset(csv_path, root_dir, output_csv):
    df = pd.read_csv(csv_path)

    valid_rows = []

    for _, row in df.iterrows():
        img_path = os.path.join(root_dir, row["image"])
        img_path = os.path.normpath(img_path)

        if os.path.exists(img_path):
            valid_rows.append(row)

    df_clean = pd.DataFrame(valid_rows)
    df_clean.to_csv(output_csv, index=False)

    print(f"Dataset limpio guardado en {output_csv}")
    return df_clean
```

Esta función lee el CSV original, verifica la existencia de cada imagen y guarda un CSV limpio.

Objetivo:

- Asegurar que el dataset contenga solo datos válidos para evitar errores durante el entrenamiento.

---

## 4. Creación del Dataset

Se define una clase personalizada de PyTorch Dataset para cargar imágenes y etiquetas.

### 4.1 Clase FollowLineDataset

```python
import cv2
import torch
from torch.utils.data import Dataset

class FollowLineDataset(Dataset):
    def __init__(self, dataframe, root_dir, transform=None):
        self.data = dataframe.reset_index(drop=True)
        self.root_dir = root_dir
        self.transform = transform

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        ...

        return image, label
```

Esta clase carga imágenes, aplica transformaciones y devuelve pares (imagen, etiqueta) donde la etiqueta es un vector [v, w].

Objetivo:

- Facilitar la carga y preprocesamiento de datos durante el entrenamiento.

---

## 5. Transformaciones de Imágenes

Se definen transformaciones para normalizar y preparar las imágenes.

### 5.1 Transformaciones aplicadas

```python
import torchvision.transforms as transforms

transform = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((66, 200)),
    transforms.ToTensor(),
    transforms.Normalize((0.5, 0.5, 0.5),
                         (0.5, 0.5, 0.5))
])
```

Las imágenes se redimensionan a 66x200 píxeles, se convierten a tensores y se normalizan.

Objetivo:

- Preparar las imágenes en el formato esperado por el modelo.

---

## 6. Modelo PilotNet

El modelo es una red neuronal convolucional inspirada en PilotNet para conducción autónoma.

### 6.1 Arquitectura del modelo

```python
import torch.nn as nn

class PilotNet(nn.Module):
    def __init__(self):
        super(PilotNet, self).__init__()

        self.conv = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2),
            nn.ReLU(),

            nn.Conv2d(24, 36, 5, stride=2),
            nn.ReLU(),

            nn.Conv2d(36, 48, 5, stride=2),
            nn.ReLU(),

            nn.Conv2d(48, 64, 3),
            nn.ReLU(),
        )

        with torch.no_grad():
            x = torch.zeros(1, 3, 66, 200)
            x = self.conv(x)
            self.flat_size = x.view(1, -1).shape[1]

        self.fc = nn.Sequential(
            nn.Linear(self.flat_size, 100),
            nn.ReLU(),
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Linear(50, 2)
        )

    def forward(self, x):
        x = self.conv(x)
        x = x.view(x.size(0), -1)
        x = self.fc(x)
        return x
```

La red consta de capas convolucionales seguidas de capas fully connected que predicen [v, w].

Objetivo:

- Aprender una representación de las imágenes que permita predecir comandos de control precisos.

---

## 7. Preparación de Datos

Se divide el dataset en conjuntos de entrenamiento y validación.

### 7.1 División y DataLoaders

```python
from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader

ROOT = "./dataset_combined"
CSV = "./dataset_combined/label.csv"

df_clean = clean_dataset(CSV, ROOT, "label_clean.csv")

# split train / val / test
train_df, temp_df = train_test_split(df_clean, test_size=0.2, random_state=42)
val_df, test_df = train_test_split(temp_df, test_size=0.5, random_state=42)

train_dataset = FollowLineDataset(train_df, ROOT, transform)
val_dataset   = FollowLineDataset(val_df, ROOT, transform)
test_dataset  = FollowLineDataset(test_df, ROOT, transform)

train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
val_loader   = DataLoader(val_dataset, batch_size=64)
test_loader = DataLoader(test_dataset, batch_size=64)
```

Se usa un 80% para entrenamiento, 10% para validación y 10% para test.

Objetivo:

- Preparar los datos para el entrenamiento eficiente con mini-batches.

---

## 8. Entrenamiento

Se entrena el modelo usando MSE Loss y optimizador Adam.

### 8.1 Bucle de entrenamiento

```python
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

model = PilotNet().to(device)

criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

epochs = 20

for epoch in range(epochs):
    model.train()
    total_loss = 0

    for imgs, labels in train_loader:
        imgs, labels = imgs.to(device), labels.to(device)

        outputs = model(imgs)
        loss = criterion(outputs, labels)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        total_loss += loss.item()

    print(f"[TRAIN] Epoch {epoch+1}: {total_loss/len(train_loader):.6f}")

    # validación
    model.eval()
    val_loss = 0

    with torch.no_grad():
        for imgs, labels in val_loader:
            imgs, labels = imgs.to(device), labels.to(device)
            outputs = model(imgs)
            loss = criterion(outputs, labels)
            val_loss += loss.item()

    print(f"[VAL]   Epoch {epoch+1}: {val_loss/len(val_loader):.6f}")

# test
model.eval()
test_loss = 0

with torch.no_grad():
    for imgs, labels in test_loader:
        imgs, labels = imgs.to(device), labels.to(device)
        outputs = model(imgs)
        loss = criterion(outputs, labels)
        test_loss += loss.item()

print(f"[TEST] Loss final: {test_loss / len(test_loader):.6f}")
```

Se entrena por 20 épocas, evaluando en validación cada época y finalmente en test.

Objetivo:

- Minimizar el error en la predicción de comandos de control.

---

## 9. Evaluación en Conjunto de Test

Después del entrenamiento, se evalúa el modelo en el conjunto de test para medir el rendimiento final.

### 9.1 Evaluación

```python
# test
model.eval()
test_loss = 0

with torch.no_grad():
    for imgs, labels in test_loader:
        imgs, labels = imgs.to(device), labels.to(device)
        outputs = model(imgs)
        loss = criterion(outputs, labels)
        test_loss += loss.item()

print(f"[TEST] Loss final: {test_loss / len(test_loader):.6f}")
```

Objetivo:

- Evaluar el rendimiento del modelo en datos no vistos durante el entrenamiento.

---

## 10. Exportación a ONNX

Finalmente, se exporta el modelo a formato ONNX para inferencia.

### 9.1 Exportación

```python
model.eval()

dummy = torch.randn(1, 3, 66, 200).to(device)

torch.onnx.export(
    model,
    dummy,
    "model.onnx",
    input_names=["image"],
    output_names=["control"],
    dynamic_axes={
        "image": {0: "batch"},
        "control": {0: "batch"}
    }
)

print("Modelo exportado como model.onnx")
```

Esto permite usar el modelo en otros frameworks o entornos.

Objetivo:

- Facilitar el despliegue del modelo entrenado.

---

## 11. Resultados

El sistema permite al robot seguir líneas de manera autónoma usando predicciones end-to-end.

Características:

- Entrenamiento eficiente con PyTorch.
- Modelo ligero basado en PilotNet.
- Exportación a ONNX para uso en producción.

---

## Videos

# Demo de Seguimiento de Línea End-to-End:
[![Follow Line End to End Demo](https://img.youtube.com/vi/H595o6eaq1c/maxresdefault.jpg)](https://youtu.be/H595o6eaq1c)