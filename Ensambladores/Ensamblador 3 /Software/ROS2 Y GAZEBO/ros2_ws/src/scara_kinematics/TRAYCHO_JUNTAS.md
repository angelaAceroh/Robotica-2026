| `scara_kinematics/juntas_nodo.py` | el nodo# «traycho» por juntas
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoAñadido del **23 de octubre de 2026**. Es el mismo botón **traycho**
| `scara_kinematics/juntas_nodo.py` | el nodo([TRAYCHO.md](TRAYCHO.md)), con los mismos ocho puntos, pero el brazo va de uno
| `scara_kinematics/juntas_nodo.py` | el nodoa otro **por juntas**, como en tus programas:
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo> «solo corrige el error cuando llega al punto, hazlo con las velocidades para
| `scara_kinematics/juntas_nodo.py` | el nodo> cada juntura que ya te dije»
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo| | traycho (TRAYCHO.md) | traycho por juntas (esto) |
| `scara_kinematics/juntas_nodo.py` | el nodo|---|---|---|
| `scara_kinematics/juntas_nodo.py` | el nodo| de punto a punto | en recta | cada junta a lo suyo: la punta hace curvas |
| `scara_kinematics/juntas_nodo.py` | el nodo| velocidades | las del launch (0.1 rad/s, 1 mm/s), las tres a la par | **fijas, las tuyas**: Alvin 14 RPM (84°/s), Simon 30 RPM (180°/s), Teodoro 60 mm/min (1 mm/s). **Desde el 24/10, Simon a 30°/s** (`vel_simon`, [CONTROL_SIN_TEMBLOR.md](CONTROL_SIN_TEMBLOR.md) §8) **y Alvin a 30°/s** (`vel_alvin`, §11: deprisa se cae la circunferencia de soporte) |
| `scara_kinematics/juntas_nodo.py` | el nodo| sin Teodoro | — | `husillo:=virtual z_inicial:=220`: solo Alvin y Simon, la z de mentira (24/10, [CONTROL_SIN_TEMBLOR.md](CONTROL_SIN_TEMBLOR.md) §9) |
| `scara_kinematics/juntas_nodo.py` | el nodo| corrige | todo el camino (`qd = J⁻¹(ẋd + K e)`) | **solo al llegar** |
| `scara_kinematics/juntas_nodo.py` | el nodo| quién lo lleva | `control_nodo` | `juntas_nodo` |
| `scara_kinematics/juntas_nodo.py` | el nodo| desde HOME | hay que meter Simon unos grados (está en su tope) | sale tal cual |
| `scara_kinematics/juntas_nodo.py` | el nodo| los puntos | +8 cm en X y −29 en Y | **P1 es el HOME** (girados −35° alrededor de P1; desde el 24/10 por la tarde, P1 en (0, −331), lejos de la base) |
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo| archivo | qué es |
| `scara_kinematics/juntas_nodo.py` | el nodo|---|---|
| `scara_kinematics/juntas_nodo.py` | el nodo| `scara_kinematics/juntas.py` | cómo va por juntas y cómo corrige al llegar, y cómo se reparte el camino (sin ROS) |
| `scara_kinematics/juntas_nodo.py` | el nodo| `scara_kinematics/juntas_nodo.py` | el nodo; se llama `control`, como `control_nodo`, para que la ventana no note el cambio |
| `scara_kinematics/juntas_nodo.py` | el nodo| `scara_kinematics/juntas_gui.py` | la ventana de `traycho_gui` heredada; solo cambia cómo reparte el camino |
| `scara_kinematics/juntas_nodo.py` | el nodo| `launch/traycho_juntas.launch.py` | placa (o la simulada) + `puente_placa` + `juntas_nodo` + la ventana |
| `scara_kinematics/juntas_nodo.py` | el nodo| `test/test_juntas.py` | 18 tests, con una placa de mentira |
| `scara_kinematics/juntas_nodo.py` | el nodo| `setup.py` | +2 líneas en `entry_points`: `juntas_nodo` y `juntas_gui` |
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoNo toca `control_nodo`, `control_placa_gui`, `control_placa.launch.py`,
| `scara_kinematics/juntas_nodo.py` | el nodo`traycho_gui` ni `puente_placa`. Lo que sí cambia, para los dos traychos:
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo- **El HOME está en (0, −270)**, no en (0, −250): el `cero` de
| `scara_kinematics/juntas_nodo.py` | el nodo  `config/micropython.yaml` (y de `alcance.py`) pasa a **−45.68°, −90.01°, 155 mm**.
| `scara_kinematics/juntas_nodo.py` | el nodo- **Las z, a 0.04 de las de tu código**, con P1 arriba del todo (TRAYCHO.md
| `scara_kinematics/juntas_nodo.py` | el nodo  §2): Teodoro baja 4.6 mm como mucho (a −5.1 mm). Antes, a un cuarto (−29.3).
| `scara_kinematics/juntas_nodo.py` | el nodo- **Teodoro no baja de −6 mm** en este modo (`TEODORO_MIN_MM` en `juntas.py`,
| `scara_kinematics/juntas_nodo.py` | el nodo  argumento `teodoro_min` del launch): lleva debajo un soporte cilíndrico.
| `scara_kinematics/juntas_nodo.py` | el nodo  Bajando a P2 (−58) chocó con él (lo paraste en −47), y bajando a P4 (−29)
| `scara_kinematics/juntas_nodo.py` | el nodo  casi (lo paraste en −10; «el soporte, en todos»). Es un tope más para
| `scara_kinematics/juntas_nodo.py` | el nodo  `juntas_nodo` y la ventana: ni el traycho ni «Ir a un punto» lo pasan.
| `scara_kinematics/juntas_nodo.py` | el nodo  **SUPUESTO**, con margen.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**P1 es el HOME** (23/10: «home es P1»; en tu código el robot arranca en P1).
| `scara_kinematics/juntas_nodo.py` | el nodoSolo trasladados, P3, P5 y P7 quedarían a 20 cm de la base, y el brazo no
| `scara_kinematics/juntas_nodo.py` | el nodollega a menos de 27 (en HOME, con Simon en su tope). Por eso la figura va
| `scara_kinematics/juntas_nodo.py` | el nodo**girada −35° alrededor de P1**, sin escalar: las mismas distancias entre
| `scara_kinematics/juntas_nodo.py` | el nodopuntos, todos a 10° o más de los topes. Caben con giros de −46° a −24°: es un
| `scara_kinematics/juntas_nodo.py` | el nodo**SUPUESTO** (`GIRO_GRADOS` en `juntas.py`). P1 va 3 mm más allá de HOME,
| `scara_kinematics/juntas_nodo.py` | el nodo(0, −273), con Simon 0.8° dentro de su tope: en el tope justo el margen no lo
| `scara_kinematics/juntas_nodo.py` | el nododeja. El traycho en recta sigue con los puntos de antes: con estos, la recta de
| `scara_kinematics/juntas_nodo.py` | el nodoP2 a P3 pasaría a 25 cm de la base.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo| | en el brazo (mm) | Alvin, Simon, Teodoro |
| `scara_kinematics/juntas_nodo.py` | el nodo|---|---|---|
| `scara_kinematics/juntas_nodo.py` | el nodo| P1 = HOME | 0, −273, 220 | 0.4°, −0.8°, −0.5 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P2 | −176, −254, 215.4 | 40.1°, −10.9°, −5.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P3 | 182, −248, 217.4 | −31.0°, −10.6°, −3.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P4 | −94, −311, 215.4 | 24.6°, −15.7°, −5.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P5 | 219, −274, 217.4 | −26.8°, −23.7°, −3.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P6 | −12, −368, 215.4 | 16.7°, −29.6°, −5.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P7 | 260, −303, 217.4 | −20.4°, −40.5°, −3.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**HOME más lejos de la base (24/10, tarde).** Con Simon en 0, el antebrazo
| `scara_kinematics/juntas_nodo.py` | el nodo**toca la base**. Sergio: «choca con la base, o sea con el robot, creo que
| `scara_kinematics/juntas_nodo.py` | el nododejamos el home muy pegado». Con Simon en −10: «ponlo como entre 5 a 10 grados
| `scara_kinematics/juntas_nodo.py` | el nodomás lejos». Primero se puso en −17.5, el punto medio. Tras el traycho de las
| `scara_kinematics/juntas_nodo.py` | el nodo16:47: «es mejor que alejemos el home por ahí 10 grados, y el home de retorno
| `scara_kinematics/juntas_nodo.py` | el nodose pegó un poquito a la base».
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo- **El HOME de ahora:** Simon en **−27.5** (`SIMON_HOME` en `juntas.py`). Con
| `scara_kinematics/juntas_nodo.py` | el nodo  x = 0, eso es **P1 = (0, −362)**, con Alvin en +13.8.
| `scara_kinematics/juntas_nodo.py` | el nodo- **La figura:** la misma, girada igual alrededor del P1 nuevo. Queda 89 mm más
| `scara_kinematics/juntas_nodo.py` | el nodo  afuera que con el HOME del 23/10, y Alvin ya no baja de −11 (antes llegaba a
| `scara_kinematics/juntas_nodo.py` | el nodo  −31, donde molestaba la circunferencia de soporte).
| `scara_kinematics/juntas_nodo.py` | el nodo- **Límite de Simon:** no pasa de **−5** hacia la base (`SIMON_MAX`, argumento
| `scara_kinematics/juntas_nodo.py` | el nodo  `simon_max` del launch, como `teodoro_min`).
| `scara_kinematics/juntas_nodo.py` | el nodo- **El cero no cambia:** la cuenta 0 de Simon sigue siendo el antebrazo contra
| `scara_kinematics/juntas_nodo.py` | el nodo  la base, así que no hace falta fijar de nuevo. Desde ahí, la primera ida a P1
| `scara_kinematics/juntas_nodo.py` | el nodo  son 27.5° de Simon y va en dos trozos de menos de 20°.
| `scara_kinematics/juntas_nodo.py` | el nodo- **Tests:** `test/test_home_lejos.py` (4).
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoLos puntos de ahora:
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo| | en el brazo (mm) | Alvin, Simon, Teodoro |
| `scara_kinematics/juntas_nodo.py` | el nodo|---|---|---|
| `scara_kinematics/juntas_nodo.py` | el nodo| P1 = HOME | 0, −362, 220 | 13.8°, −27.5°, −0.5 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P2 | −176, −343, 215.4 | 44.9°, −35.5°, −5.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P3 | 182, −337, 217.4 | −11.0°, −34.8°, −3.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P4 | −94, −400, 215.4 | 35.8°, −45.3°, −5.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P5 | 219, −363, 217.4 | −5.8°, −50.7°, −3.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P6 | −12, −458, 215.4 | 35.1°, −67.3°, −5.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo| P7 | 260, −392, 217.4 | 3.9°, −74.9°, −3.1 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo- `main_control.py`: **VEL_MAX de Alvin y Simon, de 15 a 90 y 190** (si no, la
| `scara_kinematics/juntas_nodo.py` | el nodo  placa no pasaría de 15°/s). Con `control_nodo` no cambia nada: el puente no
| `scara_kinematics/juntas_nodo.py` | el nodo  le manda más de 15.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo---
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo## 1. Cómo se usa
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo1. Los motores sueltos y la placa con su cero: Alvin y Simon en (0, −270), con
| `scara_kinematics/juntas_nodo.py` | el nodo   el antebrazo contra la base, y Teodoro arriba del todo. Si la placa no lo
| `scara_kinematics/juntas_nodo.py` | el nodo   sabe, se pone ahí **a mano** y `fijar alvin 0`, `fijar simon 0`,
| `scara_kinematics/juntas_nodo.py` | el nodo   `fijar teodoro 0`. No se lleva a motor contra la base: la transmisión de
| `scara_kinematics/juntas_nodo.py` | el nodo   Simon cede y no para. El botón Home lleva a P1, que desde el 24/10 por la
| `scara_kinematics/juntas_nodo.py` | el nodo   tarde está más afuera.
| `scara_kinematics/juntas_nodo.py` | el nodo2. Lanza:
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo```bash
| `scara_kinematics/juntas_nodo.py` | el nodocd ~/ros2_ws && source install/setup.bash
| `scara_kinematics/juntas_nodo.py` | el nodoros2 launch scara_kinematics traycho_juntas.launch.py activar:=false
| `scara_kinematics/juntas_nodo.py` | el nodo```
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo3. **Activar motores** y **traycho**.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoSin el brazo: `simular:=true`. Lo corta lo mismo que al otro: Parar (Esc), otra
| `scara_kinematics/juntas_nodo.py` | el nodoorden, «Soltar motores» o un vigilante de la placa. Si la placa suelta los
| `scara_kinematics/juntas_nodo.py` | el nodomotores, `juntas_nodo` deja lo que estuviera haciendo: al volver a activar, no
| `scara_kinematics/juntas_nodo.py` | el nodosigue solo.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoDe las demás órdenes de la ventana, `juntas_nodo` solo hace **ir** (la v que
| `scara_kinematics/juntas_nodo.py` | el nodopongas no cuenta: van las de cada junta), **home** (a **P1**, arriba: desde el
| `scara_kinematics/juntas_nodo.py` | el nodo24/10 por la tarde, (0, −331) con Simon en −17.5; antes eran los tres motores a
| `scara_kinematics/juntas_nodo.py` | el nodo0, con el antebrazo contra la base. Ver CONTROL_SIN_TEMBLOR.md) y **parar**. Salto, círculo y las demás dicen «no sale».
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo---
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo## 2. Cómo va de un punto a otro
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoPara cada punto, la cinemática inversa (rama down) da dónde tiene que acabar
| `scara_kinematics/juntas_nodo.py` | el nodocada junta. Luego:
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo1. **Ir.** Cada junta va hacia lo suyo **a su velocidad, fija**, y no se
| `scara_kinematics/juntas_nodo.py` | el nodo   corrige nada por el camino. Las juntas no se esperan entre sí: cada una para
| `scara_kinematics/juntas_nodo.py` | el nodo   cuando le falta su *anticipo*, lo que todavía anda después de mandarle 0
| `scara_kinematics/juntas_nodo.py` | el nodo   (la consigna de la placa va hasta 1.5° de motor por delante, y la orden tarda
| `scara_kinematics/juntas_nodo.py` | el nodo   unos 40 ms en llegar: **SUPUESTO**). Alvin para a ~4.9° y Simon a ~8.5°.
| `scara_kinematics/juntas_nodo.py` | el nodo2. **Asentar.** Todo a 0 hasta que las tres se paran (de 0.3 a 1.5 s).
| `scara_kinematics/juntas_nodo.py` | el nodo3. **Corregir.** Si alguna está a más de **0.4° (Alvin, Simon) o 0.25 mm
| `scara_kinematics/juntas_nodo.py` | el nodo   (Teodoro)** de su sitio, su consigna en la placa (la posición más el
| `scara_kinematics/juntas_nodo.py` | el nodo   seguimiento que manda la placa) se lleva justo a su sitio, en unos 0.4 s, y
| `scara_kinematics/juntas_nodo.py` | el nodo   se vuelve a asentar. Como mucho 3 veces.
| `scara_kinematics/juntas_nodo.py` | el nodo4. **Hecho.** Dice el error en la punta y lo que le falta a cada junta:
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo```
| `scara_kinematics/juntas_nodo.py` | el nodoir terminada | en el punto: medio 1.1 mm, maximo 1.1 mm | 1 correcciones (al acabar de ir, 18.4 mm) | le falta Alvin +0.22°, Simon -0.08°, Teodoro +0.02 mm
| `scara_kinematics/juntas_nodo.py` | el nodo```
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoLa tolerancia no puede ser menor: la placa deja de empujar a 0.3° de motor de
| `scara_kinematics/juntas_nodo.py` | el nodosu consigna (BANDA), y parada no vuelve a empujar hasta el doble. 0.3° de Alvin
| `scara_kinematics/juntas_nodo.py` | el nodoson ~2 mm en la punta con el brazo estirado.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**En planta solo va arriba; en cada punto baja y sube en vertical** (23/10:
| `scara_kinematics/juntas_nodo.py` | el nodo«no subió, quedó abajo y se chocó, debe bajar y después subir»). A cada punto
| `scara_kinematics/juntas_nodo.py` | el nodomás bajo que P1 llega por encima, con Teodoro arriba del todo; baja en vertical
| `scara_kinematics/juntas_nodo.py` | el nodo(solo Teodoro), vuelve a subir y sigue. Si al pulsar traycho la punta está
| `scara_kinematics/juntas_nodo.py` | el nodoabajo, lo primero es subir.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**El camino se parte** igual que en el otro: ninguna junta gira más de 20° en
| `scara_kinematics/juntas_nodo.py` | el nodouna ida. Pero los trozos son iguales en juntas, no sobre la recta. Desde HOME
| `scara_kinematics/juntas_nodo.py` | el nodosalen **32 idas**, unos 3 minutos con lo que tarda en asentar y corregir
| `scara_kinematics/juntas_nodo.py` | el nodo(de ellos, unos 45 s son de Teodoro bajando y subiendo a 1 mm/s; con las z a un
| `scara_kinematics/juntas_nodo.py` | el nodocuarto eran 4.5 min). La primera sale del tope de Simon hacia dentro: ahí no hace
| `scara_kinematics/juntas_nodo.py` | el nodofalta meterlo antes a mano.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**Al llegar, espera a la placa.** Parado no es llegado: Alvin, para arrancar,
| `scara_kinematics/juntas_nodo.py` | el nodonecesita que la placa le suba el PWM, y mientras tanto no se mueve. No da la ida
| `scara_kinematics/juntas_nodo.py` | el nodopor terminada hasta que la placa está a menos del doble de su banda de su
| `scara_kinematics/juntas_nodo.py` | el nodoconsigna (o pasan 3 s). Y las correcciones no van más deprisa que 1.5° de motor
| `scara_kinematics/juntas_nodo.py` | el nodocada 0.3 s: mientras un motor no arranca, la consigna no se le adelanta más de
| `scara_kinematics/juntas_nodo.py` | el nodoeso, y lo que se pidiera de más se perdería.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoEn la planta, en morado, sale el camino de verdad de todo el traycho, no
| `scara_kinematics/juntas_nodo.py` | el nodorectas: cada junta va a su velocidad, así que la punta hace curvas y no se sale
| `scara_kinematics/juntas_nodo.py` | el nodode la zona a la que llega el brazo. Antes se pintaban rectas, y la de P2 a P3
| `scara_kinematics/juntas_nodo.py` | el nodocruzaba la zona blanca (23/10: «hay una trayectoria que pasa por un punto
| `scara_kinematics/juntas_nodo.py` | el nodoblanco, eso está mal»). En verde, la ida que está haciendo.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo---
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo## 3. Probado
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoCon la placa de mentira de `test_juntas.py` (motores que dan la V, que no la
| `scara_kinematics/juntas_nodo.py` | el nododan, y con la orden tardando 80 ms): todas las idas desde HOME, cada una dentro
| `scara_kinematics/juntas_nodo.py` | el nodode la tolerancia con 1 corrección, a 1–2 mm del punto.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoCon la placa simulada (`simular:=true`), el primer intento tumbó `juntas_nodo`
| `scara_kinematics/juntas_nodo.py` | el nodoal primer aviso de nivel *warning*. rclpy no deja escribir con niveles
| `scara_kinematics/juntas_nodo.py` | el nododistintos desde la misma línea. Arreglado, con su test.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**En el brazo (23/10), con los puntos de antes:** P1 (dos idas) y P2 (1/2), a
| `scara_kinematics/juntas_nodo.py` | el nodo0.3–0.9 mm del punto con 1 o 2 correcciones. Al acabar de ir le faltaban de 16
| `scara_kinematics/juntas_nodo.py` | el nodoa 43 mm: Simon a 180°/s se pasa más de lo que se ha supuesto. Lo paró Sergio
| `scara_kinematics/juntas_nodo.py` | el nodo(Parar y Soltar) en P2 (2/2), para poner P1 en HOME.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**Segunda vez en el brazo (23/10), con P1 en HOME:** P1, P2 (1/2) y P2 (2/2)
| `scara_kinematics/juntas_nodo.py` | el nodobajando a la vez que se movía en planta. En la ida a P3, Alvin giró 16° rápido
| `scara_kinematics/juntas_nodo.py` | el nodocon Teodoro aún abajo (a 1 mm/s había subido 6 mm de 58) y **la punta se
| `scara_kinematics/juntas_nodo.py` | el nodochocó**. De ahí, bajar y subir en vertical. Alvin además se quedó dos veces a
| `scara_kinematics/juntas_nodo.py` | el nodo1.3° y 0.8° de su sitio («FUERA de la tolerancia»): se daba la ida por
| `scara_kinematics/juntas_nodo.py` | el nodoterminada mientras la placa aún lo estaba arrancando (la punta seguía
| `scara_kinematics/juntas_nodo.py` | el nodomoviéndose: medio 4.0 mm, máximo 6.2 mm en la misma llegada). De ahí, esperar
| `scara_kinematics/juntas_nodo.py` | el nodoa la placa.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoCon la placa de mentira, la vuelta entera con bajar y subir: 32 idas, unos 10
| `scara_kinematics/juntas_nodo.py` | el nodomin, todas dentro de la tolerancia con 1 o 2 correcciones, a 1.3–1.6 mm del
| `scara_kinematics/juntas_nodo.py` | el nodopunto de media. Igual con motores que tardan 0.25 o 0.4 s en arrancar. En el
| `scara_kinematics/juntas_nodo.py` | el nodobrazo, sin probar todavía.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**Tercera vez en el brazo (23/10), bajando y subiendo en vertical:** P1 y las
| `scara_kinematics/juntas_nodo.py` | el nododos idas por encima de P2, bien (0.9–2.0 mm). Bajando a P2 (−58 mm), Teodoro
| `scara_kinematics/juntas_nodo.py` | el nodochocó con el soporte cilíndrico que lleva ahora; lo paraste en −47. De ahí las
| `scara_kinematics/juntas_nodo.py` | el nodoz a un cuarto (−29 mm como mucho) y el tope de −40.
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**Cuarta vez en el brazo (23/10), con las z a un cuarto y el tope en −40:**
| `scara_kinematics/juntas_nodo.py` | el nodola placa había perdido el HOME (lo moviste con el USB desconectado y guardaba
| `scara_kinematics/juntas_nodo.py` | el nodoP2 en la flash), así que se fijó a 0, 0, 0 antes. P1, P2 (bajar y subir a
| `scara_kinematics/juntas_nodo.py` | el nodo−29), P3 (a −16) y las idas por encima de P4, bien. Bajando a P4 casi chocó
| `scara_kinematics/juntas_nodo.py` | el nodocon el soporte: lo paraste en −10 mm. De ahí las z a 0.04 y el tope en −6.
| `scara_kinematics/juntas_nodo.py` | el nodoAlvin se quedó 4 veces «FUERA de la tolerancia» en idas de paso (de 0.5° a
| `scara_kinematics/juntas_nodo.py` | el nodo1.6°: «la placa no lo acerca más»); en los puntos donde baja, sí llegó (1.6 y
| `scara_kinematics/juntas_nodo.py` | el nodo2.8 mm).
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo**En el brazo con P1 en (0, −331), Simon −17.5 (24/10, 16:47):** Alvin y Simon a 30°/s,
| `scara_kinematics/juntas_nodo.py` | el nodo`husillo:=virtual`. Salió desde Alvin −2 y Simon −17.6, e hizo las 33 idas
| `scara_kinematics/juntas_nodo.py` | el nodoenteras:
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodo| | |
| `scara_kinematics/juntas_nodo.py` | el nodo|---|---|
| `scara_kinematics/juntas_nodo.py` | el nodo| error medio de las idas | **1.43 mm** |
| `scara_kinematics/juntas_nodo.py` | el nodo| error máximo | 3.21 mm (una «FUERA de la tolerancia», Alvin −0.58°) |
| `scara_kinematics/juntas_nodo.py` | el nodo| vigilantes o AVISO | ninguno |
| `scara_kinematics/juntas_nodo.py` | el nodo| final | de vuelta en P1, a 2.5 mm |
| `scara_kinematics/juntas_nodo.py` | el nodo
| `scara_kinematics/juntas_nodo.py` | el nodoEs la primera vuelta del día que no corta la placa: antes se paraba
| `scara_kinematics/juntas_nodo.py` | el nodosiempre volviendo a P1, con el antebrazo contra la base.
| `scara_kinematics/juntas_nodo.py` | el nodoSergio lo vio entero: «está mucho mejor», pero a la vuelta el codo quedó «un
| `scara_kinematics/juntas_nodo.py` | el nodopoquito» pegado a la base. Por eso el HOME pasó a Simon −27.5, (0, −362).

**En el brazo con P1 en (0, −362), Simon −27.5 (24/10, 16:57):** salió del
HOME nuevo (con la orden `home`) e hizo las 32 idas enteras:

| | |
|---|---|
| error medio de las idas | 1.88 mm |
| error máximo | 4.32 mm (7 «FUERA de la tolerancia», todas de Alvin: −0.42° a −0.55°) |
| vigilantes o AVISO | ninguno |
| final | de vuelta en P1, a 0.9 mm |

Sergio: «perfectoooooo». Al acabar, motores sueltos: la placa guardó Alvin
13.81 y Simon −27.38.
