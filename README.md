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

### Compilar

Para compilar desde línea de comando:

```bash
cmake -B build
cmake --build build
```

CMake descarga y compila solo las dependencias (raylib y Box2D). Cada archivo `.cpp` de `src/agents/` es un ratón distinto y genera su propio ejecutable con el mismo nombre: `starter_mouse`, `keyboard_mouse` y, más adelante, el tuyo.

### Ejecutar

```bash
starter_mouse --gen 0                    # laberinto generado con semilla 0
starter_mouse --file mazes/abc.txt       # laberinto cargado desde archivo
starter_mouse --gen 0 --noise-seed 1234  # repite exactamente una ejecución anterior
```

Puedes descargar laberintos oficiales de competencias aquí:

- [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)
- [tcp4me.com - Micromouse Mazes](https://www.tcp4me.com/mmr/mazes/)

Los sensores y los motores tienen errores aleatorios (ver [Creencia y realidad](#creencia-y-realidad)), así que dos ejecuciones con el mismo laberinto no son idénticas. Al arrancar, el simulador imprime la **semilla de ruido** que usó: si pasa algo raro, puedes repetir esa misma ejecución con `--noise-seed`.

| Tecla | Acción |
| ----- | ------ |
| `R` | Inicia una corrida desde la celda de salida. |
| `F11` | Pantalla completa. |

### Reglas

El laberinto sigue el estándar **IEEE Micromouse de 16x16 celdas** (18 cm cada una). El ratón arranca en la esquina suroeste `(0, 0)`, orientado al norte. El objetivo es el cuadrado central de **2x2 celdas**.

Dispones de **5 corridas** y **300 segundos** de tiempo total. Cuenta la mejor marca individual. La competencia se corre en un laberinto sorpresa.

El tiempo de cada corrida se cuenta desde que el ratón **abandona la celda inicial** hasta que **llega a cualquiera de las cuatro celdas centrales**. Al terminar una corrida, el ratón debe **regresar autónomamente al origen**: al llegar, la siguiente corrida empieza sola.

Si el ratón se pierde o se traba, presiona `R` para volver a empezar desde la salida. Pero cuidado: **cada `R` consume una de las 5 corridas**.

Chocar no tiene penalización, pero tiene consecuencias: el ratón pierde tiempo y puede desorientarse.

### Juego limpio

Tu ratón debe resolver el laberinto como un robot real: solo con lo que perciben sus sensores. Por eso, tu código no puede:

- Usar funciones del simulador que no aparezcan en este documento.
- Acceder al estado interno del simulador.

Sí puedes recordar lo que aprendió tu ratón en las corridas anteriores. Si dudas de si algo está permitido, pregunta.

### Qué ves en pantalla

| Elemento | Significado |
| -------- | ----------- |
| Ratón naranja | Dónde está el ratón **de verdad**. |
| Contorno celeste | Dónde **cree** estar el ratón. Normalmente se superpone con el naranja; si se separa mucho, el ratón va a tomar decisiones equivocadas. |
| Rayos amarillos | Los sensores infrarrojos y lo que detectan. |
| Celdas coloreadas | Lo que pinta tu ratón con `PaintCell`. Útil para depurar. |
| `Est. error` | La diferencia entre donde el ratón está y donde cree estar: distancia en cm y ángulo en grados. |
| Cartel "LOST!" | La capa detectó que el ratón se perdió. Presiona `R` para volver a empezar desde la salida. |

## Tu ratón

### Crear tu ratón

1. Copia `src/agents/starter_mouse.cpp` a `src/agents/mi_raton.cpp` (usa el nombre de tu equipo).
2. Cambia el nombre que devuelve `GetMouseName()`.
3. Vuelve a compilar: va a aparecer el ejecutable `mi_raton`.

### Las tres funciones

Tu ratón es un archivo `.cpp` que implementa tres funciones (declaradas en `src/sim/mouse.h`):

```cpp
const char *GetMouseName();   // El nombre de tu ratón
void ResetMouse(Sim *sim);    // Se llama al presionar R
void UpdateMouse(Sim *sim);   // Se llama 1000 veces por segundo de simulación
```

`ResetMouse` se llama solo cuando presionas `R`: al arrancar, y si tienes que reiniciar el ratón a mano. Las corridas siguientes empiezan solas cuando el ratón vuelve a la salida, sin llamarla.

## La capa de navegación

Mover un ratón real es difícil: hay que controlar los motores y estimar la posición con sensores imperfectos. Para que no tengas que ocuparte de eso, te damos una **capa de navegación** (`src/agents/nav/nav.h`): tú solo tienes que decir a dónde ir. Puedes usarla tal cual o mejorarla (ver el [apéndice](#apéndice-la-capa-de-bajo-nivel)).

La capa maneja al ratón como si recorriera un grafo: las celdas son los nodos, y dos celdas vecinas están conectadas si no hay pared entre ellas. El ratón se detiene en el centro de una celda, y tú le indicas por qué conexiones seguir. Solo necesitas trabajar con **celdas**, **direcciones** y **paredes**.

### Direcciones y paredes

```cpp
enum Heading { HEADING_NORTH, HEADING_EAST, HEADING_SOUTH, HEADING_WEST };
```

Las direcciones están en orden horario: `(heading + 1) % 4` es la derecha, `(heading + 2) % 4` es atrás y `(heading + 3) % 4` es la izquierda.

Las paredes de una celda son una máscara de bits: `WALL_NORTH | WALL_EAST | WALL_SOUTH | WALL_WEST`. `HeadingToWall(heading)` convierte una dirección en su bit, y `GetNeighborCell(cell, heading)` devuelve la celda vecina en esa dirección.

Una celda es `Cell { int32_t x, y; }`, con `(0, 0)` en la esquina suroeste; `x` crece hacia el este e `y` hacia el norte.

### Funciones

| Función | Qué hace |
| ------- | -------- |
| `NavReset(sim)` | Reinicia la navegación: el ratón está en `(0, 0)` mirando al norte. Llámala en `ResetMouse`. |
| `NavUpdate(sim)` | Estima la posición y mueve los motores. Llámala al principio de `UpdateMouse`. |
| `NavIsIdle()` | `true` cuando el ratón terminó el camino y está quieto en el centro de una celda. |
| `NavIsLost()` | `true` si la capa detectó que el ratón se perdió. Se queda quieto: presiona `R`. |
| `NavGetCell()` | La celda donde el ratón cree estar. |
| `NavGetHeading()` | La dirección hacia la que el ratón cree mirar. |
| `NavGetWalls()` | Las paredes de la celda actual, vistas por los sensores al detenerse. |
| `NavFollowPath(path, count)` | Recorre un camino: cada elemento es la dirección de la próxima celda. |
| `NavSetSpeed(max_speed, acceleration)` | Velocidad de crucero y aceleración en las rectas (ver [Velocidad y riesgo](#velocidad-y-riesgo)). |

Algunos detalles importantes:

- **Planifica solo cuando el ratón está quieto.** `NavFollowPath` se ignora mientras `NavIsIdle()` sea `false`. `NavGetCell()` y `NavGetHeading()` se actualizan cuando el ratón se detiene, y `NavGetWalls()` solo es válido mientras está quieto.
- **Las paredes solo se leen donde el ratón se detiene.** Si le das un camino de varias celdas, las celdas intermedias se atraviesan sin leer sus paredes. Para explorar, avanza de a una celda; para correr por un camino conocido, dale el camino completo.
- **Las rectas son rápidas y los giros son lentos.** Los movimientos consecutivos en la misma dirección se recorren como una sola recta, acelerando y frenando una sola vez. Para girar, el ratón se detiene en el centro de la celda y gira en el lugar.
- **El camino puede terminar antes.** Si aparece una pared en el camino (o el ratón se traba), el ratón se detiene en la última celda alcanzable. Compara `NavGetCell()` con el destino para saberlo.

### Ejemplo: el starter mouse

El ratón de ejemplo (`src/agents/starter_mouse.cpp`) sigue la pared de la derecha:

```cpp
void ResetMouse(Sim *sim)
{
    NavReset(sim);
    ResetCellColors(sim);
}

void UpdateMouse(Sim *sim)
{
    NavUpdate(sim);

    // Wait until the mouse stops at a cell
    if (!NavIsIdle() || NavIsLost())
        return;

    Cell cell = NavGetCell();
    Heading heading = NavGetHeading();
    uint8_t walls = NavGetWalls();

    PaintCell(sim, cell, COLOR_CELL_VISITED);

    Heading right = (Heading)((heading + 1) % 4);
    Heading left = (Heading)((heading + 3) % 4);
    Heading back = (Heading)((heading + 2) % 4);

    Heading next;
    if (!(walls & HeadingToWall(right)))
        next = right;
    else if (!(walls & HeadingToWall(heading)))
        next = heading;
    else if (!(walls & HeadingToWall(left)))
        next = left;
    else
        next = back;

    NavFollowPath(&next, 1);
}
```

Es simple, pero lento, y en algunos laberintos puede entrar en bucles infinitos.

`UpdateMouse` se llama 1000 veces por segundo, pero tu algoritmo solo necesita pensar cuando el ratón se detiene en una celda. Aun así, cuida la complejidad de lo que calculas en cada parada: un algoritmo lento hace que la simulación vaya más lenta que el tiempo real.

### Otras funciones útiles

| Función | Qué hace |
| ------- | -------- |
| `PaintCell(sim, cell, color)` | Pinta una celda. Colores: `COLOR_CELL_DEFAULT`, `COLOR_CELL_VISITED`, `COLOR_CELL_RED`, `COLOR_CELL_GREEN`, `COLOR_CELL_BLUE`. |
| `GetCellColor(sim, cell)` | El color de una celda. |
| `ResetCellColors(sim)` | Vuelve todas las celdas al color original. |
| `SetStatusText(sim, text)` | Muestra un mensaje sobre el laberinto (`""` lo oculta). |
| `GetSimState(sim)` | El estado de la competencia: `run_number`, `run_state`, `run_time`, `run_time_best`, `time` (ver el [apéndice](#apéndice-la-capa-de-bajo-nivel)). |
| `ValidateCell(cell)`, `IsStartCell(cell)`, `IsGoalCell(cell)` | Si una celda está dentro del laberinto, es la salida o es una de las cuatro de la meta. |

## Creencia y realidad

Todo lo que te dice la capa de navegación es su **creencia**, no la realidad.

Como en un robot real, la capa no sabe dónde está el ratón: lo **estima** con sus sensores. Usa tres: **encoders** en las ruedas, que miden cuánto avanzó; un **giróscopo**, que mide cuánto giró; y **sensores infrarrojos**, que miden la distancia a las paredes.

Ningún sensor es perfecto. Las ruedas patinan un poco y su diámetro real no es exactamente el nominal, así que los encoders se equivocan un poco en cada movimiento. El giróscopo tiene un pequeño sesgo, y su error también se acumula. La capa de navegación corrige esos errores continuamente con los infrarrojos, usando las paredes como referencia.

El contorno celeste en pantalla es esa creencia, y `Est. error` es cuánto se equivoca. A la velocidad por defecto la creencia es muy confiable. Pero si el error crece demasiado, la capa puede creer que el ratón está en una celda cuando en realidad está en otra, y atribuirle las paredes a la celda equivocada.

La capa vigila esto: recuerda las paredes que vio en cada celda, y si alguna vez ve algo que contradice lo que vio antes, se da cuenta de que el ratón está perdido, lo detiene y muestra "LOST!". A partir de ahí solo queda presionar `R`.

Tu mapa del laberinto también se construye con la creencia de la capa. Si el ratón se pierde, puede que tu mapa tenga algún error: la capa no siempre se da cuenta en el mismo momento en que se equivocó.

## Velocidad y riesgo

En las rectas, la capa acelera hasta una **velocidad de crucero**, la mantiene y frena antes de llegar. Por defecto, la velocidad de crucero es 0,5 m/s y la aceleración, 2 m/s²: valores seguros. Puedes cambiarlos con `NavSetSpeed(max_speed, acceleration)`. Los motores llegan hasta 1,5 m/s, así que esa es la velocidad de crucero más alta posible.

Pero acelerar más tiene un riesgo: si los motores empujan demasiado, **las ruedas patinan** y giran más de lo que avanza el ratón. Como los encoders cuentan vueltas de rueda, la capa se equivoca al estimar la posición. Si el error crece mucho, el ratón se pierde. Cuánto arriesgar es decisión tuya.

## Tu misión

Programa un ratón que llegue al centro lo más rápido posible. Algunas ideas:

- **Flood Fill** (el más usado en competencias reales).
- **Dead-end filling**.
- **BFS / Dijkstra / A\*** sobre el mapa conocido.
- **Estrategia multi-corrida**: explorar en las primeras corridas y recorrer el camino óptimo en las últimas.

Un detalle: el camino con menos celdas no siempre es el más rápido. Las rectas largas son mucho más rápidas que los giros.

## Entrega

Debes entregar:

- El código de tu ratón.
- Un archivo `ENTREGA.md` donde documentes:
  - Nombre del equipo y del ratón.
  - Descripción del algoritmo implementado.
  - Mejor tiempo logrado (indica el laberinto que usaste: la semilla o el archivo).
  - Complejidad temporal y espacial de tu algoritmo.
  - Dificultades encontradas y cómo las resolviste.
  - Reflexión: ¿qué limitaciones tiene tu solución? ¿Qué mejorarías?

## Recomendaciones

- Prueba el **keyboard mouse** (`WASD`) con el laberinto `mazes/empty_maze.txt` para adquirir intuición de la física. Recuerda presionar `R` para iniciar la corrida.
- Empieza con el starter mouse y asegúrate de entender cómo funciona antes de intentar algo más complejo.
- Usa `PaintCell` para depurar: por ejemplo, pinta las distancias de tu flood fill o el camino que planeas.
- Prueba con múltiples semillas (`--gen`) y laberintos oficiales. Cuando algo falle, anota la semilla de ruido para poder repetirlo.
- Usa Git y haz commits con frecuencia.

## Bonus points 🚀

- Ajusta la velocidad según el tramo: por ejemplo, más rápido en las rectas largas.
- Mejora la capa de navegación (ver el apéndice): giros en arco sin detenerse, recorridos en **diagonal**…

## Referencias

- [The Fastest Maze-Solving Competition On Earth](https://www.youtube.com/watch?v=ZMQbHMgK2rw)
- [Claude Shannon — Theseus, the maze-solving mouse (film, 1952)](https://www.youtube.com/watch?v=_9_AEVQ_p74)

## Apéndice: la capa de bajo nivel

Esta sección es **opcional**. Solo la necesitas si quieres modificar la capa de navegación (`src/agents/nav/`) o reemplazarla por la tuya. Es el camino de los equipos de Micromouse reales: girar sin detenerse, recorrer diagonales, acelerar al límite de la adherencia. Es difícil: hazlo solo cuando tu ratón ya llegue al centro de forma confiable.

Puedes modificar `src/agents/nav/` directamente: el comentario al principio de `nav.cpp` explica cómo funciona. Todos los ratones de `src/agents/` la comparten, así que verifica que el starter mouse siga funcionando.

### Los motores

```cpp
void SetMouseVelocity(Sim *sim, float linear, float angular);
```

- `linear`: velocidad hacia adelante (m/s, positivo = adelante).
- `angular`: velocidad angular (rad/s, positivo = izquierda).

El controlador de los motores intenta mantener esas velocidades hasta que las cambies. Cada rueda tiene un máximo de 1,5 m/s. Si la fuerza necesaria supera la adherencia, las ruedas patinan.

### Los sensores

```cpp
const SimState *s = GetSimState(sim);
```

#### Infrarrojos

El ratón cuenta con **5 sensores de distancia** que miden la distancia en metros desde el centro del robot hasta la pared más cercana:

| Constante | Dirección |
| --------- | --------- |
| `IR_SENSOR_LEFT` | 90° izquierda |
| `IR_SENSOR_FRONT_LEFT` | 45° izquierda |
| `IR_SENSOR_FRONT` | frente |
| `IR_SENSOR_FRONT_RIGHT` | 45° derecha |
| `IR_SENSOR_RIGHT` | 90° derecha |

```cpp
s->ir_sensors[IR_SENSOR_FRONT]  // distancia al frente (m)
```

El alcance máximo es **25 cm**: si no hay pared en rango, el sensor devuelve `0.25`. La lectura tiene ruido, que crece con el cuadrado de la distancia (≈1 mm a 6 cm, ≈2 cm a 25 cm). Además, cada pared refleja la luz un poco distinto, y los postes solitarios (las columnas entre paredes) reflejan mucho menos: el sensor los ve casi al doble de su distancia real.

#### Encoders

```cpp
s->encoders[ENCODER_LEFT]   // distancia recorrida por la rueda izquierda (m, positivo = adelante)
s->encoders[ENCODER_RIGHT]  // distancia recorrida por la rueda derecha (m)
```

El promedio de ambos es cuánto avanzó el ratón, y su diferencia dividida por la trocha (`MOUSE_WHEEL_TRACK`, 70 mm) es cuánto giró. Se ponen en cero con cada `R`. Sus errores:

- El diámetro real de las ruedas difiere del nominal (≈7 mm/m, distinto para cada rueda). Es el mismo error en todas las corridas.
- Las ruedas patinan un poco todo el tiempo, y mucho si aceleras demasiado.
- No registran el deslizamiento lateral, por ejemplo al rozar una pared.

#### IMU

```cpp
s->gyroscope      // velocidad angular (rad/s, positivo = izquierda)
s->accelerometer  // Vector2 (m/s²): y = adelante, x = derecha
```

El giróscopo tiene un pequeño **sesgo** (≈0,05°/s) que cambia en cada corrida y deriva lentamente. Si lo integras para estimar la orientación, el error crece con el tiempo. El acelerómetro incluye la aceleración centrípeta de los giros.

### El estado de la competencia

```cpp
s->time           // Tiempo total (s)
s->run_number     // Número de corrida actual (1 a 5)
s->run_state      // RUNSTATE_IDLE (en la salida) / RUNSTATE_RUNNING / RUNSTATE_RETURNING
s->run_time       // Tiempo de la corrida actual (s)
s->run_time_best  // Mejor tiempo hasta ahora (0 = ninguno)
```

### Tu estimación

```cpp
void SetEstimatedPose(Sim *sim, Vector2 position, float rotation);
```

Reporta dónde cree estar tu ratón: posición en metros (`(0, 0)` es la esquina suroeste del laberinto) y rotación en radianes (0 = este, π/2 = norte). El simulador la dibuja como el contorno celeste y calcula `Est. error`: es la mejor forma de depurar tu estimación. Funciones útiles: `PositionToCell(position)`, `Vector2FromAngle(angle, length)` y `AngleDiff(source, target)`.
