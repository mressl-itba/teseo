# Level 4: Teseo

En esta práctica vas a **diseñar y programar un agente autónomo** capaz de navegar un laberinto en el menor tiempo posible.

## Contexto: el ratón que aprendió a pensar

![Logo](img/logo.jpg)

**Bell Labs, 1950.** Claude Shannon acababa de publicar *A Mathematical Theory of Communication*, el paper que fundó la teoría de la información. Pero ese año también tenía otro proyecto entre manos, más pequeño, más curioso.

Construyó un ratón de madera con un imán en la base y lo llamó **Theseus**.

Theseus se movía sobre un laberinto de 25 celdas. Debajo del tablero, un sistema de 75 relés electromagnéticos controlaba su recorrido. La primera vez que lo soltaban, el ratón exploraba el laberinto por ensayo y error. Pero una vez que encontraba la salida, memorizaba el camino. En la segunda corrida, lo recorría sin un solo error.

Shannon lo presentó en conferencias y en televisión. Lo llamó "un ejemplo de comportamiento adaptativo en máquinas". Era, en esencia, uno de los primeros dispositivos de inteligencia artificial de la historia.

Décadas después, el mundo de la robótica convirtió esa idea en una competencia oficial: **Micromouse**, organizada por el IEEE desde 1977. Robots reales navegan laberintos reales. Los mejores lo hacen en segundos.

**Tu misión**: construir el sucesor digital de Theseus, y competir contra los demás grupos por el mejor tiempo.

El laberinto ya está generado. La física, los sensores y el rendering ya funcionan. Lo que falta es lo único que importa: el algoritmo que hace que el ratón **piense**.

## El simulador

Para compilar el proyecto, instala primero las dependencias:

```bash
vcpkg install raylib box2d
```

Ejecuta el simulador con:

```bash
teseo --gen 0         # laberinto generado con semilla 0
teseo --file abc.maze # laberinto cargado desde archivo
```

Puedes descargar laberintos oficiales de competencias aquí:

- [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)
- [tcp4me.com - Micromouse Mazes](https://www.tcp4me.com/mmr/mazes/)

Presiona `R` para iniciar la primera corrida.

El laberinto sigue el estándar **IEEE Micromouse de 16x16 celdas** (18 cm cada una). El ratón arranca en la esquina suroeste `(0, 0)`, orientado al norte. El objetivo es el cuadrado central de **2x2 celdas**.

Dispones de **5 corridas** y **300 segundos** de tiempo total. Cuenta la mejor marca individual.

El tiempo de cada corrida se cuenta desde que el ratón **abandona la celda inicial** hasta que **llega a cualquiera de las cuatro celdas centrales**. Al terminar una corrida, el ratón debe **regresar autónomamente al origen** para luego iniciar la siguiente.

Si el ratón se bloquea, es posible reiniciarlo con la tecla `R`.

## El movimiento

El ratón se controla con la función:

```cpp
void SetMouseSetpoint(Sim *sim, float distance, float rotation);
```

- `distance`: metros a avanzar hacia adelante (positivo = adelante).
- `rotation`: rotación en radianes respecto de la orientación actual (positivo = izquierda). Puedes usar las constantes `TURN_CCW`, `TURN_CW` y `TURN_REVERSE`.

Cuánto más lejos pongas el setpoint, más rápido se moverá el mouse.

⚠️ **Importante**: el simulador introduce **error de odometría**. No asumas que el ratón termina exactamente donde le pediste.

## Los sensores

Puedes leer el estado de los sensores con:

```cpp
const SimState *s = GetSimState(sim);
```

### Infrarrojos

El ratón cuenta con **5 sensores de distancia** (metros) que apuntan en distintas direcciones:

| Constante | Dirección |
| --------- | --------- |
| `IR_SENSOR_LEFT` | 90° izquierda |
| `IR_SENSOR_FRONT_LEFT` | 45° izquierda |
| `IR_SENSOR_FRONT` | frente |
| `IR_SENSOR_FRONT_RIGHT` | 45° derecha |
| `IR_SENSOR_RIGHT` | 90° derecha |

```cpp
s->ir_sensors[IR_SENSOR_FRONT]  // distancia al frente (m)
s->ir_sensors[IR_SENSOR_LEFT]   // distancia a la izquierda (m)
// ...
```

El alcance máximo es **1 m**. Si no hay pared en rango, el sensor devuelve `1.0`.

### IMU

La **IMU** (Inertial Measurement Unit) mide el movimiento propio del ratón.

```cpp
s->mouse_accelerometer  // Vector2 (m/s²): y=adelante, x=derecha
s->mouse_gyroscope      // float (rad/s, CCW+): velocidad angular
```

### Setpoint

El **setpoint** es el objetivo de movimiento que le pediste al controlador con `SetMouseSetpoint(...)`. Estos campos indican cuánto falta para completar ese comando.

```cpp
s->setpoint_distance  // float (m): distancia restante (positivo = falta avanzar)
s->setpoint_rotation  // float (rad, CCW+): rotación restante
```

Puedes cambiar el setpoint en todo momento.

### Estado de la simulación

```cpp
s->time;          // Tiempo total de simulación
s->run_number;    // Número de corrida actual
s->run_state;     // idle (en celda inicial) / running / returning
s->run_time;      // Tiempo de la corrida actual
s->run_time_best; // Mejor tiempo hasta ahora
```

## Tu misión

Implementa tu propio agente en una carpeta nueva dentro de `src/`. Puedes copiar `src/starter_mouse/` como punto de partida.

### API del agente

Debes implementar la estructura `MouseDescriptor`:

```cpp
struct MouseDescriptor {
    const char *name;
    void *(*create)();                          // inicialización
    void (*destroy)(void *userdata);            // liberación de memoria
    void (*update)(void *userdata, Sim *sim);   // lógica principal, llamada cada frame
    void (*reset)(void *userdata, Sim *sim);    // llamado en la primera corrida y cuando se resetea el ratón
};
```

Dentro de `update` puedes usar estas funciones:

- `GetSimState(sim)`
- `SetMouseSetpoint(sim, distance, rotation)`
- `AngleDiff(source, target)`
- `Vector2 Vector2FromAngle(angle, length);`
- `PositionToCell(position);`
- `PaintCell(sim, cell, color);`
- `GetCellColor(sim, cell);`
- `ResetCellColors(sim);`

No puedes acceder a los campos internos de `sim` ni a las otras funciones del simulador.

### Cómo registrar tu agente

1. Copia `src/starter_mouse/` a `src/tu_equipo/`.
2. Renombra los archivos, la struct interna y la variable `MouseDescriptor`.
3. Agrega tu `.cpp` en `CMakeLists.txt`.
4. Incluye tu header en `src/main.cpp` y regístralo con `CreateMouse()`.

### Estrategias posibles

El **starter mouse** implementa un simple seguidor de pared derecha. Es útil para comenzar, pero poco eficiente y puede atascarse en bucles.

Algunas ideas mejores:

- **Flood Fill** (el más usado en competencias reales)
- **Dead-end filling**
- **Dijkstra / A\*** sobre el mapa conocido
- **Estrategia multi-corrida**: explorar en las primeras corridas y ejecutar la ruta óptima en las últimas

## Entrega

Debes entregar:

- El código de tu agente.
- Un archivo `ENTREGA.md` donde documentes:
  - Nombre del equipo y del ratón.
  - Descripción del algoritmo implementado.
  - Mejor tiempo logrado (indica la semilla del laberinto o el archivo que usaste).
  - Complejidad temporal y espacial de tu algoritmo.
  - Dificultades encontradas y cómo las resolviste.
  - Reflexión: ¿qué limitaciones tiene tu solución? ¿Qué mejorarías?

## Recomendaciones

- Prueba el **keyboard mouse** (WASD) con el laberinto `empty_maze.txt` para adquirir intuición de la física. Recuerda apretar `R` para iniciar el simulador.
- Logra que tu ratón sea robusto al **error odométrico**, que es ≈7 mm/m en distancia y ≈4° cada 90° en rotación. Puedes compensar el error de distancia con los sensores IR, y el error de rotación, integrando el dato del giróscopo.
- Empieza con el seguidor de pared y asegúrate de que funciona bien antes de intentar algo más complejo.
- Usa `PaintCell` para depurar.
- Prueba con múltiples semillas (`--gen`) y laberintos oficiales.
- Evita chocar con las paredes para no perder tiempo.
- Con la siguiente salvedad: puedes chocar suavemente con las paredes para compensar el error odométrico de rotación.
- Usa Git y haz commits con frecuencia.
- **No modifiques** el motor del simulador ni su UI.

## Bonus points 🚀

- Optimiza la trayectoria para **minimizar giros**: recorrer varias celdas seguidas en línea recta suele ser mucho más rápido.
- Optimiza los giros.
- Implementa recorridos en **diagonal sin giro** cuando el laberinto lo permita.

## Referencias

- [The Fastest Maze-Solving Competition On Earth](https://www.youtube.com/watch?v=ZMQbHMgK2rw)
- [Claude Shannon — Theseus, the maze-solving mouse (film, 1952)](https://www.youtube.com/watch?v=_9_AEVQ_p74)
