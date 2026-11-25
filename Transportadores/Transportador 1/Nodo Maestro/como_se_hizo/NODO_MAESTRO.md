# Nodo maestro de la línea de empacado

**Equipo 5 — Transportador 1** · UMNG, Robótica · Prof. Alexandra Velasco
Julián Castañeda, Juan Pablo Castro, Luis Miguel González, Sebastián Herrera, Sebastián Ramos

Estado a **29 de octubre de 2026**: integrado, compilado y probado. Las tres pruebas pasan.

---

## 1. Qué es y por qué lo hace el Equipo 5

La línea la forman **5 robots de 5 equipos distintos**. Alguien tiene que decir quién
se mueve y cuándo, porque si dos robots entran a la misma posición a la vez chocan.
Ese árbitro es el **nodo maestro**, y le tocó al Equipo 5 por ser el **transportador 1**,
el que arranca la línea.

**El maestro es un programa aparte del robot.** No mueve nada, no tiene cinemática y
no sabe cómo es ningún brazo. Solo manda órdenes por texto y escucha estados. Por eso
vive en su propio paquete y no dentro de `planta_brazo`.

Los otros 4 robots los hacen otros equipos y **todavía no han entregado ni su URDF ni
su nodo**. Mientras tanto todo se prueba con `robot_simulado.py`, que se hace pasar por
cualquiera de los 5.

---

## 2. Dónde está

Paquete **`linea_maestro`**, en `~/ros2_ws/src/linea_maestro`. Es `ament_python`.

```
linea_maestro/
├── linea_maestro/
│   ├── maestro.py            el coordinador
│   └── robot_simulado.py     finge ser cualquiera de los 5 robots
├── launch/
│   └── prueba_maestro.launch.py
├── setup.py
└── package.xml
```

Se hizo paquete nuevo **a propósito**, por dos razones:

1. El maestro coordina la línea, no el robot. Son cosas distintas y conviene que no se
   mezclen.
2. `planta_brazo` es `ament_cmake` y ya funciona. Meterle el maestro habría obligado a
   tocar su `CMakeLists.txt`.

El nombre del paquete **da igual para los otros equipos**: lo que se comparte son los
tópicos, no el paquete.

---

## 3. Contrato de comunicación

**Esto es lo único que los 5 equipos tienen que respetar. No se cambian los nombres.**

| Tópico | Tipo | Quién publica | Contenido |
|---|---|---|---|
| `/<robot>/orden` | `std_msgs/String` | el maestro | la orden a ejecutar |
| `/<robot>/estado` | `std_msgs/String` | cada robot | `listo`, `ocupado` o `error` |
| `/parada` | `std_msgs/Bool` | el maestro | `true` = todos se detienen |
| `/linea/estado` | `std_msgs/String` | el maestro | paso actual, solo para mirar |

Los 5 nombres de robot, tal cual se escriben en los tópicos:

```
transportador1   transportador2   ensamblador1   ensamblador2   ensamblador3
```

### Órdenes que entiende cada robot

| Robot | Órdenes |
|---|---|
| `transportador1` | `P0_a_P1` · `P1_a_P2` · `home` |
| `transportador2` | `P2_a_P3` · `P3_a_P4` · `home` |
| los 3 ensambladores | `ensamblar` |

### Reglas que cumple cada robot

1. Publicar su estado **cada 0,5 s**, siempre, aunque no esté haciendo nada.
2. Al recibir una orden, publicar `ocupado` **de inmediato**.
3. Al terminar, publicar `listo` **de inmediato**.
4. **Ignorar órdenes mientras está ocupado.**
5. Con `/parada = true`: detenerse y quedar en `error`.

### Dos cosas que hay que confirmarle a los otros equipos

- **Todos usan `ROS_DOMAIN_ID=42`.** Si uno usa otro, sus mensajes no llegan y el
  maestro lo da por caído.
- **El estado se publica cada 0,5 s.** El maestro declara robot caído a los **3 s** de
  silencio. Un nodo que publique cada 5 s tumba la línea él solo.

---

## 4. La secuencia

Nueve pasos, uno detrás de otro. El maestro no lanza el siguiente hasta que el robot
del actual confirma que terminó.

| # | Robot | Orden | Qué pasa |
|---|---|---|---|
| 1 | transportador1 | `P0_a_P1` | lleva la caja de la posición 0 a la 1 |
| 2 | ensamblador1 | `ensamblar` | trabaja en la posición 1 |
| 3 | transportador1 | `P1_a_P2` | lleva la caja de la 1 a la 2 |
| 4 | transportador1 | `home` | se retira y libera la zona de la 2 |
| 5 | ensamblador2 | `ensamblar` | trabaja en la posición 2 |
| 6 | transportador2 | `P2_a_P3` | lleva la caja de la 2 a la 3 |
| 7 | ensamblador3 | `ensamblar` | trabaja en la posición 3 |
| 8 | transportador2 | `P3_a_P4` | lleva la caja de la 3 a la 4, la salida |
| 9 | transportador2 | `home` | se retira |

El paso 4 no es relleno: el transportador 1 tiene que **salirse** antes de que el
ensamblador 2 entre a la posición 2.

Antes del paso 1 el maestro espera a que **los 5** robots digan `listo`. Un ciclo
completo con los robots simulados dura **25 s**.

---

## 5. Parámetros del maestro

| Parámetro | Defecto | Qué hace |
|---|---|---|
| `repetir` | `False` | `True` repite el ciclo sin parar |
| `timeout_paso` | `60.0` s | máximo para que un robot termine un paso |
| `timeout_vivo` | `3.0` s | sin recibir estado, el robot se da por caído |
| `reenvio` | `1.0` s | cada cuánto reenvía una orden no confirmada |

---

## 6. Cómo se prueba

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch linea_maestro prueba_maestro.launch.py
```

Lanza los 5 robots simulados y el maestro. Para seguir el paso actual desde otra
terminal, con el mismo `ROS_DOMAIN_ID` y el mismo `source`:

```bash
ros2 topic echo /linea/estado
```

### Prueba 2: robot caído

Con la línea corriendo, matar el nodo de un ensamblador:

```bash
pkill -f "sim_ensamblador2"
```

Debe aparecer, en unos 3 s:

```
LÍNEA DETENIDA: ensamblador2 no publica su estado hace más de 3 s
```

y los demás robots deben mostrar `Parada recibida: robot detenido`.

### Prueba 3: parada de emergencia

Con la línea a medias:

```bash
ros2 topic pub --once /parada std_msgs/msg/Bool "{data: true}"
```

Debe aparecer:

```
LÍNEA DETENIDA: Se recibió una parada de emergencia en /parada
```

---

## 7. Resultados de las pruebas

| Prueba | Resultado |
|---|---|
| Normal | Los 9 pasos en orden, ciclo completo en **25,0 s**, `La caja salió por la posición 4.` |
| Robot caído | Detectado **3,0 s** después de matarlo; los 4 robots vivos obedecieron la parada |
| Parada manual | Los 5 robots se detuvieron **en el mismo milisegundo** |

### Un bug que hubo que corregir

`maestro.py` **se caía con un traceback** la primera vez que la línea fallaba:

```
ValueError: Logger severity cannot be changed between calls.
```

`rclpy` guarda la severidad **por sitio de llamada en el código**. La función
`reportar()` tenía una sola línea:

```python
getattr(self.get_logger(), nivel)(texto)
```

usada primero con `info` y después con `error`. La segunda vez, `rclpy` lanza la
excepción y el nodo muere.

Se corrigió dándole a cada nivel su propia línea. **La lógica no se tocó**: la
secuencia, los tópicos, los tiempos y los estados son idénticos. La detección de
fallos funcionaba bien desde el principio — publicaba `/parada` y los robots paraban —;
lo único que fallaba era **imprimir el mensaje**, y el nodo moría justo ahí.

Conviene avisarle a quien escribió los archivos: ese bug salta **la primera vez que la
línea falla**, o sea exactamente cuando más falta hace el mensaje.

---

## 8. Lo que falta

### Del resto de equipos

- Su **URDF** y su **nodo**, cumpliendo el contrato de la sección 3.
- Confirmar `ROS_DOMAIN_ID=42` y los 0,5 s de publicación de estado.

### De nosotros, cuando lleguen

Cambiar el `robot_simulado` del `transportador1` por el **nodo real**: que escuche
`/transportador1/orden`, ejecute la trayectoria que toque con el jacobiano inverso, y
publique su estado cumpliendo el contrato.

Para eso hay que resolver antes un problema de nuestro robot que no es del maestro:

- **El antebrazo no existe.** Hoy detrás del eje de J3 solo va `Brazo33`, 60 mm. El
  alcance real es **263 mm**.
- La línea le asigna al transportador 1 un círculo de **Ø700**, o sea radio 350 mm.
- **El gripper tiene que medir 147 mm desde el eje de J3** para cubrirlo
  (350 − 58,15 − 145 = 147).

Mientras el gripper no exista, el transportador 1 no alcanza las posiciones de la
línea, y la integración con el robot real no tiene sentido.

---

## 9. Dos trampas del entorno, por si alguien repite las pruebas

1. **Un `ros2 launch` que muere mal deja los 5 nodos vivos.** En una prueba maté el
   `ensamblador2` de una corrida *anterior* y la línea nueva siguió como si nada,
   porque el `ensamblador2` nuevo seguía publicando. Antes de cada prueba:

   ```bash
   pgrep -af "linea_maestro/lib/linea_[m]aestro"
   ```

   Los corchetes son a propósito: sin ellos, `pgrep -f` **se encuentra a sí mismo** en
   su propio patrón, y si después se hace `kill`, se mata el shell.

2. **Para matar un nodo concreto de un launch**, mejor por parentesco (`pgrep -P` del
   PID del launch) que por patrón global, que agarra huérfanos de corridas anteriores.

---

## 10. La línea completa en Gazebo (19 de noviembre de 2026)

Paquete nuevo **`linea_planta`** (`~/ros2_ws/src/linea_planta`): los cinco robots sobre
la mesa del plano común, en su sitio, y el maestro en el mismo lanzamiento.

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch linea_planta linea_gazebo.launch.py maestro:=true
```

Sin `maestro:=true` abre solo la planta. Con `escala:=` se cambia el tamaño del plano
(por defecto 1.4: el R25 del plano frente al Ø700 del transportador 1).

Probado: el maestro hace los 9 pasos con los cinco robots en Gazebo, y el eje de cada
robot cae en su punto del plano con 0.0 mm de error.

**Ojo: el maestro todavía no mueve los robots de Gazebo.** Quien le contesta son los
`robot_simulado`. Cada robot ya escucha en Gazebo en `/<robot>/<articulación>/cmd_pos`;
falta que el nodo de cada equipo traduzca su orden a esas consignas.

Para que los modelos de los otros equipos funcionaran hubo que corregirles varias cosas
(el SCARA no se podía mover, el transportador 2 está desplazado 1.11 m, al ensamblador 2
le faltan las mallas…). Todo está explicado, equipo por equipo, en
`~/ros2_ws/src/linea_planta/README.md`, para pasárselo a cada uno.

---

## 11. Conectar a los cinco equipos, y nuestro brazo obedeciendo al maestro (20 nov 2026)

### La red

- **Todos en la misma WiFi, que NO sea la de la universidad**: las redes públicas aíslan
  a los clientes y bloquean el multicast con el que ROS 2 se descubre. Usamos el punto de
  acceso del iPhone (red 172.20.10.0/28, hasta ~13 equipos).
- Todos con **Ubuntu + ROS 2 Jazzy** y, en cada terminal, `export ROS_DOMAIN_ID=42`.
- Nadie con `ROS_LOCALHOST_ONLY=1`; `ROS_AUTOMATIC_DISCOVERY_RANGE` en `SUBNET` (el valor
  por defecto).
- Si alguien tiene el cortafuegos activo: `sudo ufw disable`.
- **Probar antes de nada**, entre dos ordenadores: en uno `ros2 multicast receive`, en
  otro `ros2 multicast send`. El primero tiene que decir `Received from <IP del otro>`.
- **Solo hay UN maestro**, el nuestro. Cada equipo lanza únicamente su robot.

### Comandos en nuestro ordenador

Terminal 1, con el brazo **en el home**:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
cd ~/robot_fisico && python3 nodo_transportador1.py --gazebo
```

Terminal 2 (en `simulados`, los equipos que aún NO estén conectados):

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch linea_planta linea_gazebo.launch.py maestro:=true \
    simulados:=transportador2,ensamblador1,ensamblador2,ensamblador3
```

Parada de emergencia: `ros2 topic pub --once /parada std_msgs/msg/Bool "{data: true}"`

`nodo_transportador1.py` cumple el mismo contrato que el `robot_simulado`. `P0_a_P1` es
la trayectoria enseñada con la caja; `P1_a_P2` **aún no tiene trayectoria** (el brazo no
se mueve y contesta a los 3 s); `home` lleva el brazo al home.

**Aviso para los otros equipos:** su nodo tiene que quedarse en `ocupado` al menos un
segundo. Si publica `ocupado` y `listo` en el mismo instante, el maestro no ve la
confirmación y reenvía la orden sin fin (nos pasó con `home` estando ya en el home).
