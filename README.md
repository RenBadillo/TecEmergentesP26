# 🦾 UR3 Digital Twin — Unity XR

> Gemelo digital interactivo del robot **Universal Robots UR3** desarrollado en Unity, con cinemática inversa, trayectorias, comunicación HTTP con Flask y soporte para realidad extendida (XR/AR/VR).

---

## 📋 Tabla de contenidos

- [Descripción general](#descripción-general)
- [Arquitectura del proyecto](#arquitectura-del-proyecto)
- [Scripts](#scripts)
  - [Núcleo IK](#núcleo-ik)
    - [UR3SimpleIKPhysical](#ur3simpleikphysical)
    - [SimpleIKAxisControlled](#simpleikaxiscontrolled)
  - [Seguridad y validación](#seguridad-y-validación)
    - [UR3KinematicSafetyChecker](#ur3kinematicsafetychecker)
    - [UR3SelfCollisionChecker](#ur3selfcollisionchecker)
    - [UR3MotionValidator](#ur3motionvalidator)
    - [UR3WorkspaceLimiter](#ur3workspacelimiter)
  - [Interacción y control](#interacción-y-control)
    - [RoutinePlacementController](#routineplacementcontroller)
    - [TrajectoryManager](#trajectorymanager)
    - [DrawTrajectoryMeta](#drawtrajectorymeta)
    - [ActivarObjeto](#activarobjeto)
  - [Comunicación](#comunicación)
    - [UR3Client](#ur3client)
    - [UR3FlaskSender](#ur3flasksender)
    - [BotonDeteccion](#botondeteccion)
    - [CameraGetButton](#cameragetbutton)
    - [CameraVideoButton](#cameravideobutton)
  - [UI y teclado](#ui-y-teclado)
    - [EnviarInputRobotProgram](#enviarinputrobotprogram)
    - [KeyboardButtonController](#keyboardbuttoncontroller)
  - [Debug y utilidades](#debug-y-utilidades)
    - [JointReaderDegrees](#jointreaderdegrees)
    - [UR3GhostPlayback](#ur3ghostplayback)
    - [UR3JointDebugger](#ur3jointdebugger)
- [Pipeline de seguridad](#pipeline-de-seguridad)
- [Comunicación con el robot real](#comunicación-con-el-robot-real)
- [Dependencias del proyecto](#dependencias-del-proyecto)
- [Configuración rápida](#configuración-rápida)
- [Flujo de uso](#flujo-de-uso)

---

## Descripción general

Este proyecto implementa un **gemelo digital (digital twin)** del robot UR3 dentro de Unity. Permite:

- Visualizar y controlar el robot virtualmente mediante **cinemática inversa (IK)**.
- Definir una **pose objetivo** arrastrando un marcador en la escena (compatible con XR/AR).
- Generar una **trayectoria interpolada** entre la posición actual y la posición objetivo.
- Enviar esa trayectoria paso a paso a un **servidor Flask** que controla el robot físico.
- Reproducir secuencias de movimiento en un **robot fantasma** (ghost) para previsualización.
- Leer en tiempo real los ángulos de los joints del robot físico vía HTTP.

---

## Arquitectura del proyecto

El sistema está organizado en **4 capas** que van desde la interfaz física del usuario hasta el robot real:

```
┌─────────────────────────────────────────────────────────────────────┐
│  CAPA 1 — Interacción (MR Interface)                                │
│                                                                     │
│  MR Interface (Hardware)        Hand Tracking                       │
│  · Cámara passthrough           · Coordenadas 3D de la mano        │
│  · Seguimiento de manos         · Tracking                         │
│  · Posición espacial            · Eventos de gesto (pinch)         │
│  · Gestión                                                          │
│                                                                     │
│  Menú (Botones)    Dibujo 3D              Servidor de Comandos      │
│  · Mov             · Recibe coordenadas   · Recibe: Menú, P1       │
│  · MoveL           · Hand tracking        · Tipo de movimiento     │
│  · MoveC           · Tracking             · Menú                   │
│  · Porta           · Genera trayectoria   · Construye estructura:  │
│  · Abrir gripper   · Guarda puntos en     │ {Move, P1}             │
│  · Cerrar gripper    memoria              │ {Move, P2}             │
│  · Simular                                │ {Gripper, Open}        │
│  · Cancelar                                                         │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────────┐
│  CAPA 2 — Procesamiento                                             │
│                                                                     │
│  Cinemática Inversa           Interpolación de trayectoria          │
│  · Captura                    · MoveJ                               │
│  · Restricciones              · MoveL                               │
│                               · MoveC                               │
│                                                  Captura de         │
│  Restricción cinemática       ◄──────────────    coordenadas        │
│  Recibe conjuntos (q1…q6)                        [accion, x, y, z] │
│                                                                     │
│  ┌─────────────────┐  ┌──────────────────┐  ┌──────────────────┐   │
│  │Límites          │  │  Autocolisión     │  │Colisión con      │   │
│  │articulares      │  │  Vi(q)∩Vj(q) ≥ 0 │  │superficie        │   │
│  │qmin ≤ qi ≤ qmax │  │                  │  │d_sup(q) ≥ 0      │   │
│  └─────────────────┘  └──────────────────┘  └──────────────────┘   │
│                                                                     │
│  ┌─────────────────┐                                                │
│  │   Workspace     │                                                │
│  │ r·‖r‖¹ + s·‖s‖ ≤ d_max                                         │
│  │ r_min ≤ ‖r‖ ≤ d │                                               │
│  └─────────────────┘                                                │
│                                                                     │
│  Planeación                                                         │
│  Conjunto ángulos q válidos → transmisión a la sesión               │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────────┐
│  CAPA 3 — Simulación (Unity Digital Twin)                           │
│                                                                     │
│  Gemelo digital en Unity          Posición/pose actual              │
│  · Recibe ángulos q + acciones    del robot real  ◄─────────────── │
│  · Ejecuta rutina                                                   │
│                                                                     │
│  Visualización de trayectoria ◄──────────────────                  │
│                                                                     │
│                               Comunicación TCP/IP                   │
│                               Envío de rutina/salida al robot real ─►
└──────────────────────────┬──────────────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────────────┐
│  CAPA 4 — Robot Real                                                │
│                                                                     │
│  Envío de comandos URScript                                         │
│          │                                                          │
│          ▼                                                          │
│  UR3 real — Ejecución física        Cámara                          │
│                                     · En efector final             │
│  Envío de pose actual ──────────────────────────────────────────►  │
│  (retroalimentación a Capa 3)                                       │
└─────────────────────────────────────────────────────────────────────┘
```

**Correspondencia capas → scripts:**

| Capa | Scripts Unity |
|---|---|
| Capa 1 — Interacción | `DrawTrajectoryMeta`, `RoutinePlacementController`, `ActivarObjeto`, `KeyboardButtonController`, `EnviarInputRobotProgram` |
| Capa 2 — Procesamiento | `UR3SimpleIKPhysical` (PreviewSolve), `UR3KinematicSafetyChecker`, `UR3SelfCollisionChecker`, `UR3WorkspaceLimiter`, `UR3MotionValidator` |
| Capa 3 — Simulación | `UR3SimpleIKPhysical` (LateUpdate), `UR3GhostPlayback`, `TrajectoryManager`, `UR3FlaskSender`, `UR3JointDebugger` |
| Capa 4 — Robot Real | `UR3Client` (lectura), `UR3FlaskSender` (POST), `BotonDeteccion`, `CameraGetButton`, `CameraVideoButton` |

**Flujo principal:**

```
Usuario mueve DragTarget
        ↓
UR3WorkspaceLimiter restringe la posición al workspace válido (tiempo real)
        ↓
RoutinePlacementController copia pose → ExecutionTarget
        ↓
Usuario presiona "Iniciar Rutina"
        ↓
UR3MotionValidator valida la trayectoria completa con GhostRobot
  ├── UR3KinematicSafetyChecker  (márgenes, singularidades, continuidad)
  └── UR3SelfCollisionChecker    (autocolisiones por física)
        ↓ (solo si es válida)
UR3FlaskSender interpola trayectoria con GhostRobot
        ↓
POST por cada paso → Servidor Flask → Robot UR3 físico
```

---

## Scripts

---

### Núcleo IK

#### UR3SimpleIKPhysical

**Archivo:** `UR3SimpleIKPhysical.cs`

Componente central del proyecto. Implementa un solucionador IK iterativo (CCD proyectado sobre ejes locales) con **dinámica física simulada**: velocidades y aceleraciones por joint, límites articulares y sincronización con el motor de física de Unity (`Physics.SyncTransforms`).

Se usa tanto en el **robot live** (ejecuta movimiento en tiempo real en `LateUpdate`) como en el **robot ghost** (resuelve IK offline con `PreviewSolveToPosition` para validación y envío).

**Campos principales:**

| Campo | Tipo | Descripción |
|---|---|---|
| `Link1`–`Link6` | `Transform` | Transforms de cada eslabón del UR3 |
| `endEffector` | `Transform` | TCP (Tool Center Point) |
| `target` | `Transform` | Posición objetivo (normalmente `ExecutionTarget`) |
| `allowMotion` | `bool` | Habilita el movimiento en `LateUpdate` |
| `routineFinished` | `bool` | Se activa cuando el efector llega al target |
| `arriveDistance` | `float` | Distancia de llegada (default: 1 cm) |
| `iterations` | `int` | Iteraciones IK por frame (default: 4) |
| `axis1`–`axis6` | `Vector3` | Eje de rotación local por link |
| `limit1`–`limit6` | `Vector2` | Límites articulares min/max en grados |
| `speed1`–`speed6` | `float` | Velocidades máximas en deg/s |
| `accel1`–`accel6` | `float` | Aceleraciones en deg/s² |

**Métodos públicos:**

```csharp
void StartRoutine()                          // activa el movimiento hacia el target
void StopRoutine()                           // detiene el movimiento
float[] GetCurrentAnglesCopy()               // devuelve copia de los ángulos actuales
void SetAnglesImmediate(float[] anglesDeg)   // aplica ángulos instantáneamente
void SetAnglesDirect(float[] qDeg)           // igual que SetAnglesImmediate (alias)
void CopyStateFrom(UR3SimpleIKPhysical other)// copia el estado completo desde otro robot
bool PreviewSolveToPosition(Vector3 pos, int iterations, float tolerance)
// resuelve IK sin animación, devuelve true si convergió
```

> `PreviewSolveToPosition` es la función clave para el sistema de seguridad: permite resolver la IK del robot ghost sin afectar al live robot, verificar el resultado y solo entonces ejecutar el movimiento real.

---

#### SimpleIKAxisControlled

**Archivo:** `SimpleIKAxisControlled.cs`

Solucionador IK alternativo, más ligero, sin física ni límites articulares. Útil para objetos de prueba o visualizaciones simples. Se ejecuta en `LateUpdate`.

| Campo | Tipo | Descripción |
|---|---|---|
| `joints` | `Transform[]` | Joints Joint1–Joint5 |
| `endEffector` | `Transform` | Joint6 / efector final |
| `target` | `Transform` | Posición objetivo del IK |
| `localAxes` | `Vector3[]` | Eje de rotación local por joint |
| `iterations` | `int` | Iteraciones por frame (default: 8) |
| `stepSizeDeg` | `float` | Ángulo máximo por iteración (default: 2°) |
| `stopDistance` | `float` | Distancia de parada (default: 5 mm) |

---

### Seguridad y validación

> Estos cuatro scripts forman el **pipeline de seguridad** que se ejecuta antes de enviar cualquier movimiento al robot físico. Ver también la sección [Pipeline de seguridad](#pipeline-de-seguridad).

#### UR3KinematicSafetyChecker

**Archivo:** `UR3KinematicSafetyChecker.cs`

Verifica condiciones cinemáticas sobre un vector de ángulos. Todos los métodos devuelven `bool` y un `string reason` con la causa del fallo.

| Campo | Tipo | Descripción |
|---|---|---|
| `qMinDeg` / `qMaxDeg` | `float[6]` | Límites articulares (default: ±180°) |
| `jointMarginDeg` | `float` | Margen de seguridad antes del límite (default: 10°) |
| `wristSingularityDeg` | `float` | Umbral de singularidad de muñeca en q5 (default: 5°) |
| `elbowSingularityDeg` | `float` | Umbral de singularidad de codo en q3 (default: 7°) |
| `maxJointJumpDeg` | `float` | Salto articular máximo entre pasos consecutivos (default: 40°) |

**Métodos públicos:**

```csharp
bool CheckJointMargins(float[] qDeg, out string reason)
// verifica que ningún joint esté cerca de su límite

bool CheckSingularityHeuristic(float[] qDeg, out string reason)
// detecta configuraciones próximas a singularidades de muñeca y codo

bool CheckContinuity(float[] qPrevDeg, float[] qNowDeg, out string reason)
// verifica que el salto entre dos pasos consecutivos no sea brusco
```

---

#### UR3SelfCollisionChecker

**Archivo:** `UR3SelfCollisionChecker.cs`

Detecta **autocolisiones** entre los eslabones del robot usando `Physics.ComputePenetration`. Ignora por defecto los links adyacentes (que siempre se tocan).

| Campo | Tipo | Descripción |
|---|---|---|
| `links` | `LinkColliderEntry[]` | Array de pares (nombre, Collider) por eslabón |
| `ignoreAdjacentLinks` | `bool` | Ignora colisiones entre links consecutivos (default: true) |

```csharp
// Estructura de cada entrada
public class LinkColliderEntry {
    public string linkName;
    public Collider collider;
}
```

**Método público:**

```csharp
bool HasSelfCollision(out string reason)
// devuelve true si hay penetración entre dos links no adyacentes
```

---

#### UR3MotionValidator

**Archivo:** `UR3MotionValidator.cs`

Orquesta la validación completa de una trayectoria usando el robot ghost. Para cada muestra interpolada del segmento ejecuta: convergencia IK → márgenes articulares → singularidades → continuidad → autocolisión.

| Campo | Tipo | Descripción |
|---|---|---|
| `liveRobot` | `UR3SimpleIKPhysical` | Robot de origen (estado inicial) |
| `ghostRobot` | `UR3SimpleIKPhysical` | Robot de validación (no visible) |
| `selfCollisionChecker` | `UR3SelfCollisionChecker` | Checker de colisiones (opcional) |
| `kinematicChecker` | `UR3KinematicSafetyChecker` | Checker cinemático |
| `samplesPerSegment` | `int` | Muestras por segmento (default: 12) |
| `previewIterationsPerSample` | `int` | Iteraciones IK por muestra (default: 80) |
| `previewTolerance` | `float` | Tolerancia IK (default: 1.5 cm) |

**Métodos públicos:**

```csharp
bool ValidateSingleTarget(Vector3 targetPosition, out string reason)
// valida el segmento desde la posición actual hasta un único target

bool ValidateWaypointPath(List<Vector3> waypoints, out string reason)
// valida una ruta completa de N waypoints consecutivos
```

---

#### UR3WorkspaceLimiter

**Archivo:** `UR3WorkspaceLimiter.cs`

Restringe la posición del `DragTarget` al **workspace operativo** del UR3 en tiempo real. Las restricciones se expresan en coordenadas locales de un `WorkspaceFrame` anclado a la base del robot, combinando una caja AABB y un anillo radial en el plano XZ.

| Campo | Tipo | Descripción |
|---|---|---|
| `target` | `Transform` | Objeto a restringir (normalmente `DragTarget`) |
| `workspaceFrame` | `Transform` | Transform fijo en la base del robot |
| `clampInRealTime` | `bool` | Activa el clamp en `LateUpdate` |
| `restrictToFront` | `bool` | Impide posiciones detrás de la base |
| `minX` / `maxX` | `float` | Rango de alcance frontal (m) |
| `maxAbsZ` | `float` | Alcance lateral máximo (m) |
| `minY` / `maxY` | `float` | Rango de altura (m) |
| `minRadiusXZ` / `maxRadiusXZ` | `float` | Anillo radial en plano XZ (m) |
| `drawWorkspaceGizmo` | `bool` | Visualiza el workspace en el editor |

**Métodos públicos:**

```csharp
bool IsWorldPointInside(Vector3 worldPoint, out string reason)
// comprueba si un punto está dentro del workspace

Vector3 ClampWorldPoint(Vector3 worldPoint)
// proyecta un punto al punto válido más cercano

bool ValidateSegment(Vector3 a, Vector3 b, out string reason)
// verifica que todos los puntos de un segmento estén en el workspace

bool ValidatePath(List<Vector3> waypoints, out string reason)
// verifica todos los waypoints y segmentos entre ellos
```

> Los gizmos del workspace (caja verde + círculos radiales) son visibles en la vista Scene del editor al seleccionar el objeto.

---

### Interacción y control

#### RoutinePlacementController

**Archivo:** `RoutinePlacementController.cs`

Controla la **interacción XR** para colocar el objetivo del robot. El usuario arrastra una esfera (`DragTarget`); al soltarla, la pose se copia a `ExecutionTarget` y aparece un botón para confirmar el movimiento.

| Campo | Tipo | Descripción |
|---|---|---|
| `dragTarget` | `Transform` | Esfera que maneja el usuario |
| `executionTarget` | `Transform` | Empty que recibe la pose final |
| `routineCanvas` | `GameObject` | Canvas con el botón de confirmación |
| `startButton` | `Button` | Botón "Iniciar Rutina" |
| `ur3IK` | `UR3SimpleIKPhysical` | Referencia al robot principal |
| `showButtonOnlyAfterRelease` | `bool` | Oculta el botón mientras el objeto está agarrado |
| `copyDragTargetAgainOnButtonPress` | `bool` | Re-copia la pose al pulsar el botón (modo seguro) |

**Métodos principales:**

```csharp
void OnTargetSelected()      // llamar desde XR Interactable → SelectEntered
void OnTargetUnselected()    // llamar desde XR Interactable → SelectExited
void OnStartRoutinePressed() // conectar al onClick del botón
```

---

#### TrajectoryManager

**Archivo:** `TrajectoryManager.cs`

Gestiona una lista de **waypoints** y los visualiza con un `LineRenderer` y prefabs de marcadores.

| Campo | Tipo | Descripción |
|---|---|---|
| `waypoints` | `List<Vector3>` | Lista de puntos de la trayectoria |
| `lineRenderer` | `LineRenderer` | Componente de línea en escena |
| `waypointVisualPrefab` | `GameObject` | Prefab esférico para marcar visualmente cada punto |

**Métodos públicos:**

```csharp
void AddWaypoint(Vector3 point)  // agrega un punto y actualiza la línea
void ClearWaypoints()            // borra todos los puntos y destruye los marcadores
```

---

#### DrawTrajectoryMeta

**Archivo:** `DrawTrajectoryMeta.cs`

Permite al usuario **dibujar una trayectoria en el aire** usando el gesto de pinch del dedo índice con el hand tracking de Meta Quest (OVR). El primer punto de la trayectoria es siempre la posición actual del efector final (`Link6`), garantizando continuidad con el estado del robot. Al soltar el pinch, la trayectoria puede imprimirse en consola y/o guardarse como CSV.

| Campo | Tipo | Descripción |
|---|---|---|
| `rightHand` | `OVRHand` | Referencia a la mano derecha del SDK de Meta |
| `rightHandSkeleton` | `OVRSkeleton` | Esqueleto de la mano para obtener el dedo índice |
| `trajectoryRoot` | `Transform` | Marco de referencia del robot (coordenadas locales) |
| `link6` | `Transform` | Efector final — punto de inicio de cada trazo |
| `lineRenderer` | `LineRenderer` | Visualización de la trayectoria en escena |
| `minDistanceBetweenPoints` | `float` | Distancia mínima entre puntos capturados (default: 1 cm) |
| `clearOnNewStroke` | `bool` | Borra la trayectoria anterior al iniciar un nuevo trazo |
| `printCoordinatesWhenFinish` | `bool` | Imprime los puntos locales en consola al soltar el pinch |
| `saveCsvWhenFinish` | `bool` | Guarda la trayectoria en un CSV al soltar el pinch |
| `csvFileName` | `string` | Nombre del archivo (se guarda en `Application.persistentDataPath`) |

**Métodos públicos:**

```csharp
void ClearTrajectory()
// borra todos los puntos y reinicia el LineRenderer

List<Vector3> GetLocalTrajectoryPoints()
// devuelve los puntos en coordenadas locales del trajectoryRoot

List<Vector3> GetWorldTrajectoryPoints()
// devuelve los puntos en coordenadas mundo

List<Vector3> GetTrajectoryRelativeToStart()
// devuelve los puntos relativos al primer punto (desplazamientos)

void PrintLocalTrajectory()   // log en consola de los puntos locales
void PrintWorldTrajectory()   // log en consola de los puntos mundo
void SaveTrajectoryToCSV()    // guarda CSV con columnas index, x, y, z
```

**Formato del CSV exportado:**

```
index,x,y,z
0,0.123456,0.456789,0.789012
1,0.125000,0.460000,0.790000
...
```

> El archivo se guarda en `Application.persistentDataPath`, que en Meta Quest corresponde a la carpeta de almacenamiento de la app en el dispositivo.

---

#### ActivarObjeto

**Archivo:** `ActivarObjeto.cs`

Utilidad simple para **mostrar u ocultar** el modelo 3D del UR3 y su botón de cierre desde cualquier evento de UI o XR.

| Campo | Tipo | Descripción |
|---|---|---|
| `ur3` | `GameObject` | Modelo del robot UR3 |
| `botonCerrar` | `GameObject` | Botón de cierre que acompaña al modelo |

```csharp
void MostrarUR3()   // activa ur3 y botonCerrar
void OcultarUR3()   // desactiva ur3 y botonCerrar
```

---

#### UR3Client

**Archivo:** `UR3Client.cs`

Consulta el **servidor Flask** cada 100 ms para obtener los ángulos actuales del robot físico y mostrarlos en un `TextMeshPro`.

| Campo | Tipo | Descripción |
|---|---|---|
| `jointsText` | `TextMeshProUGUI` | Texto donde se muestran los valores |
| `url` *(privado)* | `string` | `http://192.168.4.127:5000/joints` |

**Formato JSON esperado del servidor:**

```json
{
  "type": "joints",
  "values": [j1, j2, j3, j4, j5, j6]
}
```

> ⚠️ Cambiar la IP en el campo `url` según la red local.

---

#### UR3FlaskSender

**Archivo:** `UR3FlaskSender.cs`

Genera una **trayectoria interpolada** (Lerp lineal) desde la posición actual del efector hasta el objetivo, resuelve la IK en el robot fantasma para cada muestra y envía cada paso al servidor Flask mediante `POST`.

| Campo | Tipo | Descripción |
|---|---|---|
| `serverUrl` | `string` | URL del endpoint Flask (ej. `http://172.22.26.38:5000/movimiento`) |
| `liveRobot` | `UR3SimpleIKPhysical` | Robot principal (estado de origen) |
| `ghostRobot` | `UR3SimpleIKPhysical` | Robot fantasma para cálculo previo |
| `targetFinal` | `Transform` | Posición de destino |
| `samplesPerSegment` | `int` | Número de pasos de la trayectoria (default: 10) |
| `previewIterations` | `int` | Iteraciones IK por muestra (default: 80) |
| `previewTolerance` | `float` | Tolerancia de convergencia IK (default: 1.5 cm) |
| `idFigura` | `string` | Identificador de la figura/movimiento |
| `mostrarJsonEnConsola` | `bool` | Activa log del JSON en consola |
| `txtUltimoEnviado` | `TextMeshProUGUI` | Muestra el último paso en grados en UI |

**Método público:**

```csharp
void EnviarTargetActualAlServidor()
```

**Formato JSON enviado por paso:**

```json
{
  "id_figura": "figura_001",
  "id_movimiento": "figura_001",
  "paso": 3,
  "j1": 0.523,
  "j2": -1.047,
  "j3": 1.571,
  "j4": -0.785,
  "j5": 1.047,
  "j6": 0.0
}
```

> Los valores de joints se envían en **radianes**.

---

#### BotonDeteccion

**Archivo:** `BotonDeteccion.cs`

Activa o desactiva la **detección por visión computacional** en el servidor Flask mediante llamadas GET. Pensado para conectarse a botones de UI en la escena XR.

| Campo | Tipo | Descripción |
|---|---|---|
| `baseUrl` | `string` | IP base del servidor Flask (ej. `http://172.22.24.202:5000`) |

```csharp
void ActivarDeteccion()    // GET /activar_deteccion
void DesactivarDeteccion() // GET /desactivar_deteccion
```

---

#### CameraGetButton

**Archivo:** `CameraGetButton.cs`

Solicita **un frame de imagen** al servidor Flask y lo proyecta sobre un `Renderer` en la escena (simulando una pantalla). Usa shader `Unlit/Texture` para máxima fidelidad de color.

| Campo | Tipo | Descripción |
|---|---|---|
| `pantallaRenderer` | `Renderer` | Objeto 3D donde se mostrará la imagen |
| `url` | `string` | Endpoint del snapshot (ej. `.../snapshot.jpg`) |

```csharp
void PedirFrameCamara()
// dispara un GET al endpoint y aplica la imagen al material de pantallaRenderer
```

> Se agrega `?t=Time.time` a la URL para evitar caché del navegador/WebRequest.

---

#### CameraVideoButton

**Archivo:** `CameraVideoButton.cs`

Versión de streaming de `CameraGetButton`: solicita frames repetidamente a un FPS configurable para simular **video en tiempo real** sobre un plano 3D en la escena XR.

| Campo | Tipo | Descripción |
|---|---|---|
| `pantallaRenderer` | `Renderer` | Plano donde se renderiza el video |
| `url` | `string` | Endpoint del snapshot Flask |
| `fps` | `float` | Frecuencia de refresco (default: 8 fps) |

```csharp
void ToggleVideo()
// alterna entre iniciar y detener el loop de video
```

> El loop usa una corrutina con `WaitForSeconds(1f / fps)` entre frames. Detener el video llama a `StopCoroutine` sobre esa corrutina.

---

### UI y teclado

#### EnviarInputRobotProgram

**Archivo:** `EnviarInputRobotProgram.cs`

Puente entre un `TMP_InputField` de la UI y un componente `RobotProgram`. Al presionar el botón asociado, toma el texto del campo y llama a `SetTextAndRestart` en el programa del robot.

| Campo | Tipo | Descripción |
|---|---|---|
| `inputField` | `TMP_InputField` | Campo de texto de la UI |
| `robotProgram` | `RobotProgram` | Componente que interpreta y ejecuta el programa |

```csharp
void EnviarTexto()
// lee inputField.text y llama a robotProgram.SetTextAndRestart(texto)
```

---

#### KeyboardButtonController

**Archivo:** `KeyboardButtonController.cs`

Abre el **teclado no nativo de MRTK** (`NonNativeKeyboard`) en la escena XR para permitir entrada de texto desde el headset, asociado a un `TMP_InputField`.

| Campo | Tipo | Descripción |
|---|---|---|
| `inputField` | `TMP_InputField` | Campo donde se vuelca el texto al finalizar |
| `distancia` | `float` | Distancia frente al usuario (referencia, no aplicada automáticamente) |

```csharp
void AbrirTeclado()
// activa NonNativeKeyboard y llama a PresentKeyboard con el texto actual del inputField
```

> Requiere que `NonNativeKeyboard` de MRTK esté presente en la escena como singleton. Depende de `Microsoft.MixedReality.Toolkit.Experimental.UI`.

---

#### JointReaderDegrees

**Archivo:** `JointReaderDegrees.cs`

Lee los ángulos actuales de los joints en **grados** a partir de `localEulerAngles.z`. Normaliza al rango `(-180, 180]`.

| Campo | Tipo | Descripción |
|---|---|---|
| `joints` | `Transform[]` | Array con los 6 transforms de los joints |

```csharp
float[] GetJointAnglesDeg()
```

> **Tecla de debug:** `J` en Play imprime los ángulos en la consola.

---

#### UR3GhostPlayback

**Archivo:** `UR3GhostPlayback.cs`

Reproduce una secuencia pregrabada de ángulos en el **robot fantasma**, útil para previsualizar un movimiento antes de ejecutarlo en el robot real.

| Campo | Tipo | Descripción |
|---|---|---|
| `liveRobot` | `UR3SimpleIKPhysical` | Robot principal (referencia) |
| `ghostRobot` | `UR3SimpleIKPhysical` | Robot fantasma que se anima |
| `waitPerStep` | `float` | Tiempo de espera entre pasos (default: 0.2 s) |

```csharp
void PlayJointSequence(List<float[]> sequenceDeg)
// cada elemento es un float[6] en grados
```

---

#### UR3JointDebugger

**Archivo:** `UR3JointDebugger.cs`

Muestra en pantalla los ángulos actuales del robot gemelo en un `TextMeshPro`.

| Campo | Tipo | Descripción |
|---|---|---|
| `robot` | `UR3SimpleIKPhysical` | Robot del que se leen los ángulos |
| `txtGemeloFinal` | `TextMeshProUGUI` | Texto donde se imprimen los valores |

```csharp
void MostrarJointsActualesDelGemelo()
```

---

## Pipeline de seguridad

Antes de enviar cualquier movimiento al robot físico, el sistema ejecuta las siguientes comprobaciones en orden sobre el **robot ghost** (sin mover el robot real):

```
Para cada muestra de la trayectoria interpolada:
│
├── 1. PreviewSolveToPosition()
│         ¿Convergió la IK dentro de la tolerancia?
│
├── 2. CheckJointMargins()
│         ¿Algún joint está demasiado cerca de su límite mecánico?
│
├── 3. CheckSingularityHeuristic()
│         ¿q3 o q5 están cerca de 0° (singularidades de codo/muñeca)?
│
├── 4. CheckContinuity()
│         ¿El salto entre el paso anterior y el actual supera maxJointJumpDeg?
│
└── 5. HasSelfCollision()           (opcional, requiere colliders configurados)
          ¿Hay penetración entre links no adyacentes?

Si cualquier check falla → la trayectoria se rechaza con un mensaje de razón.
Solo si todos pasan → UR3FlaskSender envía los pasos al robot real.
```

Adicionalmente, **UR3WorkspaceLimiter** actúa en tiempo real mientras el usuario arrastra el `DragTarget`, impidiendo físicamente que se defina un objetivo fuera del alcance operativo antes de que empiece la validación.

---

## Comunicación con el robot real

El proyecto se comunica con dos servidores Flask que pueden correr en la misma o distinta máquina:

**Servidor de control del robot:**

| Endpoint | Método | Script | Descripción |
|---|---|---|---|
| `/joints` | `GET` | `UR3Client` | Devuelve los ángulos actuales del UR3 en JSON |
| `/movimiento` | `POST` | `UR3FlaskSender` | Recibe un paso de trayectoria y lo ejecuta |

**Servidor de visión / cámara:**

| Endpoint | Método | Script | Descripción |
|---|---|---|---|
| `/snapshot.jpg` | `GET` | `CameraGetButton`, `CameraVideoButton` | Devuelve un frame JPEG de la cámara |
| `/activar_deteccion` | `GET` | `BotonDeteccion` | Activa el pipeline de detección CV |
| `/desactivar_deteccion` | `GET` | `BotonDeteccion` | Desactiva el pipeline de detección CV |

Configura las IPs según tu red local en cada script:

| Script | Campo | IP por defecto en el código |
|---|---|---|
| `UR3Client` | `url` | `192.168.4.127:5000` |
| `UR3FlaskSender` | `serverUrl` | `172.22.26.38:5000` |
| `BotonDeteccion` | `baseUrl` | `172.22.24.202:5000` |
| `CameraGetButton` | `url` | `172.22.24.202:5000` |
| `CameraVideoButton` | `url` | `172.22.114.181:5000` |

---

## Dependencias del proyecto

Extraídas del `Packages/manifest.json`. El proyecto está construido sobre **Meta XR SDK 83** y **Unity 2022.3 LTS** con target Android (Meta Quest).

### Paquetes principales

| Paquete | Versión | Descripción |
|---|---|---|
| `com.meta.xr.sdk.all` | 83.0.0 | Meta XR SDK completo (hand tracking, interacción, audio, haptics) |
| `com.meta.xr.mrutilitykit` | 83.0.0 | Utilidades MR de Meta (scene understanding, anchors) |
| `com.unity.xr.oculus` | 4.5.4 | Plugin XR de Oculus para Unity |
| `com.unity.xr.management` | 4.5.4 | Gestión del subsistema XR |
| `com.unity.textmeshpro` | 3.0.7 | TextMeshPro para UI |
| `com.unity.timeline` | 1.7.6 | Timeline de Unity |
| `com.unity.visualscripting` | 1.9.4 | Visual Scripting de Unity |
| `com.itisnajim.socketiounity` | git | Cliente Socket.IO para Unity (vía GitHub) |
| `com.unity.nuget.newtonsoft-json` | 3.2.1 | JSON serialization (dependencia de SocketIO) |

### Dependencias de Meta XR SDK (resueltas automáticamente)

| Paquete | Versión |
|---|---|
| `com.meta.xr.sdk.core` | 83.0.0 |
| `com.meta.xr.sdk.audio` | 83.0.0 |
| `com.meta.xr.sdk.voice` | 83.0.0 |
| `com.meta.xr.sdk.haptics` | 83.0.0 |
| `com.meta.xr.sdk.platform` | 83.0.0 |
| `com.meta.xr.sdk.interaction` | 83.0.0 |
| `com.meta.xr.sdk.interaction.ovr` | 83.0.0 |

### Dependencias externas no incluidas en el manifest

Estos paquetes deben instalarse manualmente o estar presentes en el proyecto:

| Dependencia | Uso |
|---|---|
| **MRTK NonNativeKeyboard** | Teclado XR en `KeyboardButtonController.cs` (`Microsoft.MixedReality.Toolkit.Experimental.UI`) |
| **Servidor Flask** | Python 3.8+, Flask, `ur_rtde` o librería equivalente para control del UR3 |

> El registro de paquetes de Meta XR SDK usa `https://packages.unity.com`. Asegúrate de tener el scope registry de Meta configurado en Unity: `Edit → Project Settings → Package Manager`.

---

## Configuración rápida

1. Clona el repositorio y ábrelo en Unity.
2. Asigna los prefabs y transforms en el Inspector según la jerarquía descrita en [Arquitectura](#arquitectura-del-proyecto).
3. Actualiza las IPs en `UR3Client.cs` y `UR3FlaskSender.cs`.
4. Inicia el servidor Flask en el PC conectado al UR3.
5. Da Play en Unity.

---

## Flujo de uso

```
[CAPA 1] Usuario gesticula pinch con la mano derecha (Meta Quest)
            → DrawTrajectoryMeta captura la trayectoria desde Link6
            ó
            → RoutinePlacementController: arrastra DragTarget y suelta
              UR3WorkspaceLimiter restringe la posición automáticamente

[CAPA 2] UR3MotionValidator valida la trayectoria completa sobre GhostRobot
            → CheckJointMargins + CheckSingularityHeuristic + CheckContinuity
            → HasSelfCollision (si hay colliders configurados)
            → Si falla: se muestra la razón y se cancela

[CAPA 3] UR3FlaskSender genera N pasos interpolados con PreviewSolveToPosition
            → GhostRobot visualiza la trayectoria prevista
            → LiveRobot ejecuta el movimiento en la simulación (LateUpdate)
            → UR3Client recibe la pose actual del robot real y actualiza UI

[CAPA 4] Cada paso se envía vía POST al servidor Flask → UR3 físico ejecuta
            → Cámara en efector devuelve frames a CameraVideoButton
            → BotonDeteccion activa/desactiva detección por visión
```

---

> Proyecto desarrollado para control y visualización de robots industriales en entornos de realidad extendida.
