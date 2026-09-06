# RobotController

Controlador de firmware para ESP32 que gobierna una cadena de joints mediante bus **CAN (TWAI)**, exponiendo una interfaz web integrada que sirve de monitor de mensajes, herramienta de calibración y panel de mando del brazo robótico.

El micro actúa como **punto de acceso WiFi**: al encenderse crea una red propia desde la que se accede a la GUI embebida sin cables ni routers.

---

## Características

- **Monitor CAN tiempo real**: lista los mensajes que circulan por el bus (ID, DLC y datos) directamente en la web.
- **Monitor FAULT**: bandeja de errores con `id de dispositivo` y `código de error` con nombre, sin trama raw. Deduplicado: un mismo id no vuelve a aparecer hasta pasados 10 segundos.
- **TX Manual**: envía tramas arbitrarias a cualquier ID CAN con posición y velocidad.
- **Calibración por joint**: envía comando de calibración con `pos`, `ratio` y rango `[mín, máx]`.
- **Control del brazo**: edición de ángulos por joint (±5°) con **Aplicar** (envía `MoveJ` a 30°/s hacia la nueva posición) y **Cancelar** (restaura valores medidos). Gestión de puntos guardados (`MoveJ` / `MoveL`), cinemática directa (FK) y pad direccional.
- **Parada de emergencia (seta)**: botón grande que envía `USER_ERROR` por CAN y lleva la máquina de estados a `FAULT`.
- **Watchdog CAN**: heartbeat de comando vacío (`DLC=0`) que el controlador emite de forma continua (periodo configurable en `main.cpp`).
- **Validación de configuración**: la configuración cinemática (parámetros DH, rotaciones, planar y calibración) se carga desde un JSON en la web, se valida y se aplica en caliente.
- **Gateo por máquina de estados**: los movimientos desde la GUI (`/api/control`, `/api/movej`) se rechazan salvo en el estado `IDLE/STANDBY`; el resto de la API permanece accesible.
- **LEDs de estado** en la GUI que reflejan la máquina de estados: `INIT` (calibración pendiente), `IDLE`, movimiento y `FAULT`.

---

## Requisitos

- ESP32 (cualquier variante `esp32`)
- [ESP-IDF v5.1.6](https://docs.espressif.com/projects/esp-idf/) (o compatible v5.x)
- Herramienta de programación tipo `idf.py`

---

## Hardware y pines

| Función        | Pin / parámetro                     | Detalle                          |
|----------------|-------------------------------------|----------------------------------|
| CAN (TWAI) TX  | `GPIO21`                            | 500 kbit/s                        |
| CAN (TWAI) RX  | `GPIO22`                            | Modo `NORMAL`, filtro *accept-all* |
| WiFi (AP)      | SSID `RobotController`              | Clave `12345678`                 |
| AP IP          | `192.168.4.1`                       | Puerto web `80`                  |
| UART bridge    | UART2, RX `GPIO16`, TX `GPIO17`     | 115200 baud (de momento **desactivado**; fuente incluida pero no registrada en el build) |

> Nota: los pines CAN pueden cambiarse en `communication_handler.cpp` (`TWAI_GENERAL_CONFIG_DEFAULT`). Los credenciales de la AP, en `wifi.h`.

---

## Arquitectura del firmware

El código vive en `main/`, con un componente auxiliar `components/matrix_math`.

| Módulo | Responsabilidad |
|--------|-----------------|
| `communication_handler.cpp` | Driver CAN/TWAI: arranque, cola de recepción, registro de servicios, envío (`sendMessage`, `sendError`, `sendWatchdog`), tarea de recepción con despacho por ID. |
| `can_msg_queue.c` | Anillo de mensajes CAN recibidos consumido por el monitor web. |
| `fault_monitor.cpp` | Almacén de fallos recibidos (id + código + timestamp) con deduplicación por id. |
| `web.cpp` | Servidor HTTP embebido y la API JSON. Sirve `www/index.html` embebido. |
| `wifi.c` | Arranque del acceso point WiFi (AP). |
| `state_machine.cpp` | Máquina de estados jerárquica (raíz + submáquina dentro de IDLE). |
| `sm_events.cpp` | Eventos thread-safe que la GUI/CAN postean y la máquina consume. |
| `Arm.cpp` / `Joint.cpp` | Modelo del brazo: joints (posición, velocidad, calibración, registro) y movimientos `MoveJ`, `MoveJTo`, `MoveL`, `RotateJoint`. |
| `CinematicsUtils.cpp` | Cinemática directa (FK) y Jacobiana usando los parámetros DH. |
| `Config.cpp` / `Config.hpp` | Estado global de configuración cargada. |
| `joints_storage.cpp` | Almacén de puntos guardados (ángulos + pose). |
| `uart_bridge.cpp` | Puente UART (desactivado). |
| `components/matrix_math` | Operaciones de matrices 4x4 para cinemática. |

La GUI está embebida en la flash como `EMBED_FILES` (`www/index.html`); es una única página de tema oscuro con tres pestañas: *Opciones especiales*, *Control* y *Config*.

---

## Protocolo CAN

### Formato de ID

- **Servicios globales**: el ID es directamente el código de comando (p. ej. `CMD_ERROR = 0x0001`).
- **Servicios por joint**: el ID codifica el joint en el byte alto y el comando en el bajo: `(joint << 8) | comando` (p. ej. `0x0205` = joint 2, move target).

### Comandos

| Definición        | Valor | Descripción                          |
|-------------------|-------|--------------------------------------|
| `CMD_ERROR`       | 0x01  | Señalización de error.               |
| `CMD_START`       | 0x02  | —                                    |
| `CMD_CALIBRATION` | 0x03  | Calibración del joint.               |
| `CMD_ANNOUNCE`    | 0x04  | Anuncio periódico de presencia (joint sin calibrar). |
| `CMD_MOVE_TARGET` | 0x05  | Orden de movimiento (`pos` + `vel`). |
| `CMD_PAUSE`       | 0x06  | Pausa.                               |
| `CMD_RESUME`      | 0x07  | Reanudar / recuperar de `FAULT`.     |
| `CMD_STATUS`      | 0x08  | Estado del joint (eco de posición/velocidad). |
| `CMD_WATCHDOG`    | 0x09  | Heartbeat del controlador (`DLC=0`, emitido por `watchdog_task`). |

### Códigos de error

| Definición                  | Valor |
|-----------------------------|-------|
| `NO_ERROR`                  | 0x00  |
| `ENCODER_ERROR`             | 0x01  |
| `COLISION_ERROR`            | 0x02  |
| `OUT_OF_RANGE_ERROR`        | 0x03  |
| `MOVE_FAILURE_ERROR`        | 0x04  |
| `CALIBRATION_FAILURE_ERROR` | 0x05  |
| `COMMUNICATION_LOST_ERROR`  | 0x06  |
| `USER_ERROR`                | 0x07  |
| `UNKNOWN_ERROR`             | 0xAA  |

### Tramas de datos

**`MOVE_TARGET`** (8 bytes):
```
data[0..3]  pos (float, grados)
data[4..7]  vel (float)
```

**`CALIBRATION`** (8 bytes, valores ×100):
```
data[0..1]  pos   (uint16_t)
data[2..3]  ratio (int16_t, firmado)
data[4..5]  min   (uint16_t)
data[6..7]  max   (uint16_t)
```

**`STATUS`** (8 bytes): eco `pos`/`vel` como floats, con el que el controlador actualiza las lecturas de cada joint.

**`ERROR`** (1 byte): `data[0]` es el código de error.

### Registro de joints

Los joints no calibrados emiten `CMD_ANNOUNCE` repetidamente. El controlador lo registra vía `registerJointService(CMD_ANNOUNCE, ...)` y marca el joint como `registered`. **Antes de enviar cualquier calibración se verifica que todos los joints involucrados hayan sido registrados**; si alguno no lo está, la API responde con un error y no se envía nada.

---

## Máquina de estados

Jerárquica: una máquina raíz y una submáquina activa dentro del estado `IDLE`.

```
Raíz:      INIT -> IDLE -> FAULT
Submáquina (dentro de IDLE): STANDBY <-> ACTION <-> PAUSE
```

- `INIT -> IDLE`: se produce al calibraar (`Arm::calibrate`), momento en que el sistema queda listo.
- `IDLE -> FAULT`: al pulsar la seta de la GUI (`STOP`), que envía `USER_ERROR` por CAN y postea el evento `STOP`. (Recibir `CMD_ERROR` por CAN solo alimenta el monitor de fallos, no cambia el estado.)
- `FAULT -> IDLE`: al recibir un `CMD_RESUME` por CAN.
- `STANDBY -> ACTION`: evento `MOVE` (movimiento iniciado desde la GUI).
- `ACTION -> STANDBY`: cuando `Arm::isMoving()` es falso (fin de `MoveL` por posición objetivo, timeout de `MoveJ` en función de la duración calculada, o timeout de `RotateJoint` de 3 s).
- `ACTION -> PAUSE` / `PAUSE -> ACTION`: por eventos `PAUSE` / `RESUME` (vía CAN).
- `PAUSE -> STANDBY`: fin del movimiento en pausa.

Los movimientos se **rechazan** salvo en `IDLE/STANDBY` (`StateMachine::canDoAction()`). Los **LEDs de la GUI** reflejan el estado:

| Estado | LED 1 (estado)         | LED 2 (movimiento) |
|--------|------------------------|--------------------|
| INIT   | Parpadea naranja       | Apagado            |
| IDLE   | Verde fijo             | Apagado            |
| ACTION | Verde (si IDLE)        | Parpadea naranja   |
| FAULT  | Rojo                   | Rojo               |

---

## API web

La API es JSON sobre HTTP en el puerto 80.

| Método | Ruta                  | Descripción                                          |
|--------|-----------------------|------------------------------------------------------|
| GET    | `/`                   | GUI (index.html embebido).                           |
| GET    | `/api/messages`       | Mensajes CAN del anillo (`{id, dlc, data, t}`).      |
| GET    | `/api/faults`         | Fallos registrados (`{id, err, t}`).                 |
| GET    | `/api/joints`         | Posiciones actuales de los joints.                   |
| GET    | `/api/fk`             | Cinemática directa (`{x, y, z}`).                    |
| GET    | `/api/config`         | Número de joints configurado.                        |
| GET    | `/api/points`         | Puntos guardados.                                    |
| GET    | `/api/state`          | Estado de la máquina (`{root, sub}`).                |
| POST   | `/api/send`           | TX manual: envía `pos`/`vel` a un ID.                |
| POST   | `/api/send_cal`       | TX calibración manual (pos/ratio/min/max).           |
| POST   | `/api/control`        | Movimiento de joint por delta o comando de dirección.|
| POST   | `/api/movej`          | `MoveJ` a ángulos concretos (`{angles, vel}`).       |
| POST   | `/api/error`          | Envía `USER_ERROR` por CAN (seta).                  |
| POST   | `/api/config/validate`| Valida y carga el JSON de configuración.             |
| POST   | `/api/config/apply`   | Aplica la configuración y ejecuta la calibración.    |
| POST   | `/api/points`         | Guarda un punto (posiciones).                        |
| POST   | `/api/points/load`    | Ejecuta `MoveJ`/`MoveL` a un punto guardado.         |
| POST   | `/api/points/delete`  | Elimina un punto.                                    |

---

## Configuración

La pestaña *Config* carga un JSON con esta estructura (ver `config/conf.json`):

```json
{
  "DH": [
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]
  ],
  "rot": [1],
  "planar": [],
  "calibration": [
    { "pos": 0.0, "ranges": [0.0, 0.0], "ratio": 1.0 }
  ]
}
```

- **DH**: filas `[theta, alpha, d, a]` de la tabla Denavit–Hartenberg.
- **rot / planar**: índices de joints rotatorios / planares.
- **calibration**: `pos`, `ranges` (`[mín, máx]`) y `ratio` por joint.

Flujo: `Set config` valida la estructura y la deja cargada en memoria; `Calibration` aplica los parámetros a la cinemática y dispara la calibración CAN de los joints. La calibración **aborta** (con error en la GUI) si algún joint involucrado no está registrado en el bus.

---

## Compilación y flasheo

Requiere el entorno IDF preparado (`export.sh` o extensiones de VSCode):

```bash
# Preparar el entorno (ajusta la ruta a tu instalación)
source $IDF_PATH/export.sh

# Compilar
idf.py build

# Flashear y monitor (sustituye PORT por tu dispositivo, p. ej. /dev/ttyUSB0)
idf.py -p PORT flash monitor
```

Tras el arranque, la GUI estará disponible en:

```
http://192.168.4.1
```

---

## Estructura de directorios

```
RobotController/
├── CMakeLists.txt
├── sdkconfig
├── components/
│   └── matrix_math/          # Bibliotecas de álgebra de matrices
├── config/
│   └── conf.json             # Configuración de ejemplo (DH/calibración)
└── main/
    ├── main.cpp              # app_main, servicios CAN registrados
    ├── communication_handler.cpp / .hpp
    ├── can_msg_queue.c / .h
    ├── fault_monitor.cpp / .hpp
    ├── sm_events.cpp / .hpp
    ├── state_machine.cpp / .hpp
    ├── Arm.cpp / .hpp
    ├── Joint.cpp / .hpp
    ├── CinematicsUtils.cpp / .hpp
    ├── Config.cpp / .hpp
    ├── joints_storage.cpp / .h
    ├── web.cpp / .h
    ├── wifi.c / .h
    ├── uart_bridge.cpp / .hpp   # Desactivado
    └── www/index.html           # GUI embebida
```