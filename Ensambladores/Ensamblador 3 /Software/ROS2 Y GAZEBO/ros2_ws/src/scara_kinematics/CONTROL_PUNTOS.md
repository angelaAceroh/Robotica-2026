# `control_puntos`: el ejercicio de las tres fases, con tu brazo

Añadido del **21 de octubre de 2026**. Es el ejemplo de control cinemático que
pasó Sergio (el de otro SCARA, con `L1 = 0.20`, `a1 = 0.165`, `a2 = 0.135`),
hecho para este brazo:

- **Fase 1**: una trayectoria por varios puntos.
- **Fase 2**: te pide puntos por la terminal, en cm, hasta que escribas `salir`.
- **Fase 3**: las cuatro gráficas.

La diferencia de fondo: **el ejemplo simula** (integra su propio modelo con
Euler) y **este manda al brazo**. El control lo hace `control_nodo`, el mismo de
[CONTROL.md](CONTROL.md) y [CONTROL_PLACA.md](CONTROL_PLACA.md), con la punta
**medida**: los encoders de la placa, o Gazebo.

| archivo | qué es |
|---|---|
| `scara_kinematics/control_puntos.py` | el nodo: las tres fases |
| `test/test_control_puntos.py` | 16 tests (sin ROS) |
| `setup.py` | +1 línea en `entry_points`: `control_puntos` |

No toca nada de lo que ya había.

---

## 1. Cómo se lanza

En **dos terminales**. La fase 2 lee el teclado, y un `ros2 launch` no deja
escribir a sus nodos.

**Terminal 1**, el control (uno de los dos):

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch scara_kinematics control_placa.launch.py                 # el brazo (main_control.py en la placa)
ros2 launch scara_kinematics control_placa.launch.py simular:=true   # sin el brazo: la placa simulada
ros2 launch scara_kinematics control.launch.py tarea:=nada           # Gazebo
```

**Terminal 2**, el ejercicio:

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run scara_kinematics control_puntos --ros-args \
    --params-file install/scara_kinematics/share/scara_kinematics/config/scara_urdf3.yaml
```

Espera a que `control_nodo` dé señales y a que acabe lo que esté haciendo, y
empieza la fase 1.

| parámetro | por defecto | qué es |
|---|---|---|
| `puntos` | `[15, -20, 20,  19, -24, 19,  23, -21, 18,  25, -26, 19,  20, -28, 18,  17, -24, 19,  15, -20, 20]` | la trayectoria de la fase 1, en cm, de tres en tres |
| `v` | `0` | mm/s de cada recta; 0 = la de `control_nodo` (20 con el brazo, 40 en Gazebo) |
| `graficas` | `~/control_puntos.png` | dónde se guardan las gráficas; al lado, un `.csv` con todo lo apuntado |
| `ventana` | `true` | `false` = solo las guarda, sin abrir la ventana |

Por ejemplo, `-p v:=10.0 -p "puntos:=[30.0, -30.0, 12.0, 20.0, -35.0, 10.0]"`.
Los números llevan punto decimal: ROS no deja pasar `30` a una lista de
decimales.

Los **puntos por defecto** están dentro de la zona que alcanza el brazo con los
recorridos que diste el 21/10 (Alvin −60..60°, Simon −280..0°, Teodoro
−135..0 mm; `config/micropython.yaml`), en la rama del codo en la que está el
brazo, alrededor de tu HOME (0, −250). El primero está cerca de HOME, y **de uno
a otro ninguna junta gira más de 20°** (lo pediste así; como mucho 17.4°). Las rectas
entre ellos también caben. Las dos cosas las comprueba un test. Vuelve al
primero para cerrar la vuelta.

**Ctrl+C** en la fase 1 o en la 2 manda `parar` a `control_nodo` (el brazo se
queda regulando donde está) y pasa a las gráficas.

---

## 2. Las fases

### Fase 1: la trayectoria por puntos

Para cada punto, una orden `ir X Y Z` a `control_nodo`: una **recta** desde
donde está la punta, con perfil trapecio, a la velocidad `v`. Si en algún
tramo una junta pasaría de su techo, `control_nodo` la hace más lenta. Espera a
que acabe cada una y dice cómo ha ido:

```
=== FASE 1: trayectoria por 7 puntos ===
   P1 = [2, -28, 20] cm
   ...
   P1: ir 20.0 -280.0 200.0
   P1: llega en 4.7 s | error en el camino: medio 2.155 mm, maximo 7.169 mm | al final 3.53 mm
   P2: ir 80.0 -320.0 190.0
   P2: llega en 16.7 s | error en el camino: medio 1.284 mm, maximo 7.772 mm | al final 0.30 mm
```

(es el brazo de verdad, §5). Si un punto no llega, lo dice y sigue con el
siguiente. **Si la placa suelta los motores** (salta un vigilante), corta la
fase 1 y dice por qué:

```
   P1: cortado: parado: regulo donde estoy | la placa ha soltado los motores: Teodoro 0.25 s empujando (de PWM 450 a 920) y no se ha movido ni 0.012 mm: ...
   se acaba la fase 1: con los motores sueltos no se mueve nada
```

y en la fase 2 no manda nada hasta que los vuelvas a activar (botón «Activar
motores» de la ventana).

### Fase 2: los puntos a mano

```
Ingrese nuevo punto [X, Y, Z] en cm (o 'salir'): 30, -30, 12
   T1: ir 300.0 -300.0 120.0
   T1: llega en 7.5 s | error en el camino: medio 0.266 mm, maximo 1.082 mm | al final 0.01 mm
Ingrese nuevo punto [X, Y, Z] en cm (o 'salir'): 70 0 10
   T2: NO LLEGA, no se manda. P = [700.0, 0.0, 100.0] fuera del alcance del SCARA (radio 700.0 mm)
Ingrese nuevo punto [X, Y, Z] en cm (o 'salir'): salir
```

Vale `30, -30, 12`, `[30, -30, 12]` o `30 -30 12`. Antes de mandar nada,
comprueba que el punto cabe con los **topes del `control_nodo` que esté en
marcha**: se los pide al arrancar, junto con los techos de velocidad. Con el
brazo, esos topes son los recorridos de los motores; en Gazebo, los del CAD.
Después, `control_nodo` lo vuelve a comprobar. Si lo rechaza, sale su motivo.

### Fase 3: las gráficas

De **las dos fases** (el ejemplo solo dibuja la 1). La fase 2 va sombreada, y
cada punto marcado con su nombre (P1…, T1…).

1. **Seguimiento de la punta**: x, y, z pedidas (discontinua) y medidas
   (continua), en cm.
2. **Error de la punta** (pedida − medida), por eje y su norma, en cm.
3. **Posición de las juntas**: θ1 y θ2 en grados, d3 en mm. Van en dos franjas,
   porque grados y mm no comparten eje.
4. **Velocidad que manda el control** a cada junta, con sus techos (±). También
   en dos franjas: °/s y mm/s.

Se guardan en `~/control_puntos.png`, y al lado `~/control_puntos.csv`: una
fila por dato que llegó (`t_s, campo, a, b, c`) y las marcas de cada punto.

---

## 3. Cómo sabe lo que hace `control_nodo`

No calcula el control: lo escucha.

| qué | de dónde |
|---|---|
| que una orden ha empezado (`ir: 12.3 s ...`), ha acabado (`ir terminada \| ...`) o no sale (`«ir ...» no sale: ...`) | lo que escribe `control_nodo` en su consola, que también va por `/rosout` |
| que está libre para empezar | `scara/control/estado` dice `regulando ...` |
| la punta pedida, la medida y el error (mm) | `scara/control/datos/punta_ref`, `punta_real` y `error` |
| la velocidad que manda a las juntas (°/s, °/s, mm/s) | `scara/control/datos/qd` |
| dónde están las juntas | `/joint_states` (la placa a través de `puente_placa`, o Gazebo), con los nombres de junta que use `control_nodo` |
| los techos y los topes | los parámetros del propio `control_nodo` (se llama `control`) |
| que la placa ha soltado los motores, y por qué | lo que escribe `puente_placa` en `/rosout` (`PARADO ... motores sueltos`, y el `ERROR` o `VIGILANTE` de antes) |

Si `control_nodo` no contesta en 5 s a una orden, dice «¿Está en marcha el
launch?» y sigue.

---

## 4. Lo que cambia respecto al ejemplo, y por qué

| | el ejemplo | aquí |
|---|---|---|
| quién mueve | nadie: `q = q + q̇·dt` sobre su propio modelo | el brazo (o Gazebo), por `control_nodo` |
| la ley | `q̇ = J⁻¹(ẋd + K e)`, K = 15 | la misma, con la punta **medida**: K = 3 con el brazo, 5 en Gazebo |
| el error | solo sale de la saturación (su modelo es perfecto) | el de verdad: encoders, inercia, rozamiento, retrasos del USB |
| saturación | `np.clip` junta a junta | se escalan **las tres** por el mismo factor |
| cerca de una singularidad | `np.linalg.solve`; con el brazo casi estirado pide velocidades enormes y el `clip` las corta | mínimos cuadrados amortiguados |
| fase 1 | interpola en **juntas** entre puntos: la punta hace curvas | **rectas** en el espacio, con perfil trapecio |
| tiempo de cada tramo | `max(Δq / q̇max)`: la junta más lenta, a tope desde el primer instante | lo pone `v` y el perfil; si alguna junta pasaría de su techo, más lento |
| fase 2 | recta, con `T = max(Δq/q̇max, 0.5 s)` | recta, igual que la fase 1 |
| cinemática inversa | una sola rama del codo; `d3 = L1 − z` | las dos ramas y los topes de cada junta |
| punto que no llega | `np.clip(cos θ2)` lo lleva al borde sin decir nada | lo dice antes de mover, con el motivo |
| empieza | dando por hecho que el brazo está en el primer punto | desde donde esté el brazo |
| gráficas | solo la fase 1; rad, m y cm en el mismo eje | las dos fases; cada unidad en su eje |

Cosas del ejemplo que no se han cogido, y por qué:

- **El `clip` junta a junta cambia la dirección de `q̇`.** Si solo se satura una
  junta, la punta sale de la recta. Escalando las tres, la punta va más lenta
  pero por el mismo camino.
- **En la fase 2, `T = max(Δq/q̇max)` no garantiza el techo.** Sale de los
  ángulos del principio y del final, pero en una recta las juntas no van a
  velocidad constante: a mitad de camino alguna puede pasar de `q̇max`, el `clip`
  la corta y la punta se desvía. `control_nodo` revisa las velocidades en toda
  la recta y, si alguna junta pasaría de su techo, la hace más lenta. (Los topes
  de posición, en cambio, solo los comprueba en el punto final; en marcha, la
  ley no deja que una junta siga contra su tope.)
- **`d3 = L1 − z` y una sola rama** valen para su robot. En este, z sube con d3
  (`z = z_ref + d3`), y la rama depende de dónde esté el codo.
- **Dos puntos seguidos iguales** dan `T_seg = 0`, y `s = t_local / T_seg`
  divide por cero. Aquí, un punto donde ya está la punta no se manda («ya está
  ahí»).
- `Puntos_cm / 101.0 if False else Puntos_cm / 100.0` y `print(h:=...)` son
  restos que no hacen nada.

---

## 5. Probado

**Lo primero, con la calibración de antes del 21/10 por la tarde** (los puntos
de entonces: `[38, -22, 12]`, ...). Con `control_placa.launch.py simular:=true` (tu `main_control.py` de verdad,
con motores simulados detrás de un puerto falso; K = 3, techos 0.2 rad/s y
8 mm/s, rectas a 20 mm/s) y `control_puntos` con la fase 2 tecleada por un
script:

| | tarda | error medio en el camino | máximo | al llegar |
|---|---|---|---|---|
| P1 (desde el arranque) | 27.9 s | 0.28 mm | 2.27 mm | 0.23 mm |
| P2 | 14.2 s | 0.35 mm | 2.12 mm | 0.52 mm |
| P3 | 11.7 s | 0.29 mm | 1.11 mm | 0.20 mm |
| P4 | 12.2 s | 0.24 mm | 0.95 mm | 0.38 mm |
| P5 (= P1) | 12.1 s | 0.30 mm | 1.07 mm | 0.85 mm |
| T1 `30, -30, 12` | 7.5 s | 0.27 mm | 1.08 mm | 0.01 mm |
| T4 `[25, -35, 15]` | 6.2 s | 0.23 mm | 0.82 mm | 0.85 mm |

En la fase 2 también se probó: `abc` (no son números), `70 0 10` (no llega: no
se manda) y un punto donde ya estaba (no se manda). Las gráficas salieron con las dos
fases.

Con **Gazebo** (`control.launch.py gui:=false tarea:=nada`: K = 5, techos
1.5 rad/s y 8 mm/s, rectas a 40 mm/s), con los puntos P1, P2 y P1 otra vez, y
`30, -30, 12` en la fase 2:

| | tarda | error medio | máximo | al llegar |
|---|---|---|---|---|
| P1 (desde el brazo estirado) | 8.4 s | 0.020 mm | 0.037 mm | 0.03 mm |
| P2 | 7.1 s | 0.023 mm | 0.044 mm | 0.04 mm |
| P3 (= P1) | 7.1 s | 0.023 mm | 0.045 mm | 0.04 mm |
| T1 | 3.8 s | 0.038 mm | 0.088 mm | 0.06 mm |

Gazebo no tiene rozamiento ni USB por medio: de ahí que el error sea diez veces
menor que con la placa simulada. **Ctrl+C** a mitad de la fase 1: `control_nodo`
recibe el `parar` («parado: regulo donde estoy»), salen las gráficas y el nodo
se cierra. Los tests: `python3 -m pytest src/scara_kinematics/test/test_control_puntos.py`.

**Con el brazo de verdad (21/10, tarde)**, con los recorridos y los sentidos
que diste, el cero en tu HOME y los puntos de alrededor de HOME:
`control_placa.launch.py k:=2 vel_max:=0.1` (techo de Teodoro, 1 mm/s) y la
fase 1 entera, sin vigilantes:

| | tarda | error medio | máximo | al llegar |
|---|---|---|---|---|
| P1 | 4.7 s | 2.2 mm | 7.2 mm | 3.53 mm |
| P2 | 16.7 s | 1.3 mm | 7.8 mm | 0.30 mm |
| P3 | 16.7 s | 0.9 mm | 5.0 mm | 1.93 mm |
| P4 | 16.7 s | 1.8 mm | 10.7 mm | 0.53 mm |
| P5 | 16.7 s | 0.8 mm | 4.7 mm | 0.64 mm |
| P6 | 16.7 s | 2.6 mm | 8.3 mm | 0.47 mm |
| P7 | 16.7 s | 0.8 mm | 6.2 mm | 0.69 mm |

Antes de esa, dos veces se cortó porque Teodoro se atascaba subiendo; el
firmware ahora sube solo el PWM si un motor no arranca (CONTROL_PLACA.md §9).

Es diez veces más que con la placa simulada: la z va casi exacta, pero Alvin
avanza a tirones (necesita PWM 700 para moverse) y el plano oscila 5–10 mm. Cada
tramo tarda 16.7 s porque Teodoro va a 1 mm/s. Las gráficas, en
`~/control_puntos_brazo.png`. Lo que hubo que cambiar hasta llegar ahí está en
[CONTROL_PLACA.md](CONTROL_PLACA.md) §9. En `control_puntos`:

- **Cortar la fase 1 si la placa suelta los motores.** Antes seguía mandando
  puntos con el brazo suelto (no se movía nada, pero los errores salían de
  100 mm).
- **La gráfica 3 salía vacía con la placa**: buscaba las juntas con los nombres
  por defecto (`Simon_joint`...), y la placa publica los del YAML (`juntura1`,
  ...). Ahora coge los de `control_nodo`.
- **Un `ir` de 0 s** (la punta a menos de un mm) acaba en el mismo instante en
  que empieza, y se quedaba esperando 15 s su final. Ya no.

**Lo que sigue siendo supuesto** es el `cero` de Teodoro
([CONTROL_PLACA.md](CONTROL_PLACA.md) §9). El de Alvin y Simon sale de tu HOME.
Si el de Teodoro no es, la z de la ventana y la de verdad no coinciden, pero el
brazo no se sale de sus recorridos.

**Desde HOME, antes de lanzar**, mete Simon unos grados hacia dentro (en HOME
está en 0, su tope, y `control_nodo` rechazaría el primer `ir`):
CONTROL_PLACA.md §3.
