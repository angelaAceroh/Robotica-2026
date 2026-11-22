#!/usr/bin/env bash
# abrir.sh - arranca y para el SCARA: los seis nodos, con Gazebo o con RViz.
#
#   ./abrir.sh                        gazebo + rviz + los 6 nodos + la ventana
#   ./abrir.sh -b                     lo mismo, en segundo plano: la terminal queda libre
#   ./abrir.sh parar                  cierra todo lo que haya abierto
#   ./abrir.sh estado                 dice que esta corriendo ahora mismo
#   ./abrir.sh registro               sigue el log del arranque en segundo plano
#   ./abrir.sh rviz                   RViz, sin fisica
#   ./abrir.sh brazo                  solo los 6 nodos, sin dibujo
#   ./abrir.sh -n gazebo              sin recompilar
#   ./abrir.sh -sv gazebo             sin la ventana del brazo
#   ./abrir.sh -sh rviz               sin la placa (no abre el puerto serie)
#   ./abrir.sh -h                     esta ayuda
#
# Los setup.bash de ROS usan variables sin definir: nada de "set -u" aqui.

WS="$HOME/ros2_ws"
PAQUETES="scara_kinematics scara_urdf"
LOG="$WS/log/scara_sesion.log"
PIDF="$WS/log/scara_sesion.pid"
COMPILAR=1
VENTANA=1
HARDWARE=1
FONDO=0

rojo()  { printf '\033[31m%s\033[0m\n' "$*"; }
verde() { printf '\033[32m%s\033[0m\n' "$*"; }
neg()   { printf '\033[1m%s\033[0m\n'  "$*"; }

ayuda() { sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; }

# Al matar un "ros2 launch" sus nodos hijos pueden quedarse huerfanos, asi que
# para saber que sigue vivo hay que buscarlos uno a uno en vez de fiarse del pid
# del lanzador. robot_state_publisher entra en la lista porque tambien se queda
# huerfano; si usas otro robot_state_publisher para algo mas, esto lo matara.
#
# "gz-sim" es imprescindible: lo que se lanza es un envoltorio
# "sh -c ruby .../gz sim ...", y el servidor de verdad (gz-sim-main) y la
# ventana (gz-sim-gui-client) son hijos suyos con OTRO nombre. Si solo matas el
# envoltorio, el servidor sobrevive huerfano con el mundo cargado y la siguiente
# sesion se encuentra con "Another world of the same name is running": el robot
# se le entrega al servidor viejo y el mundo que ves aparece vacio.
PATRON="ros2 launch scara_kinematics|gz sim|gz-sim|rviz2|parameter_bridge|ros_gz_sim|scara_kinematics/lib/scara_kinematics|robot_state_publisher"

# Nunca hay que tocar la propia cadena de procesos: si escribes en la terminal
# un comando que mencione "rviz2" o "gz-sim" (un grep, sin ir mas lejos), pgrep
# -f encuentra ESE shell y el script se suicidaria a media faena.
ancestros() {
  local p=$$
  while [ -n "$p" ] && [ "$p" -gt 1 ]; do
    printf '%s ' "$p"
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
  done
}

# Los zombies (estado Z) ya estan muertos, solo esperan a que su padre los
# recoja: si se cuentan como vivos, "parar" cree que ha fallado.
vivos() {
  local p estado mios
  mios=" $(ancestros) "
  for p in $(pgrep -f "$PATRON"); do
    case "$mios" in *" $p "*) continue ;; esac
    estado=$(ps -o stat= -p "$p" 2>/dev/null | tr -d ' ')
    case "$estado" in
      ''|Z*) continue ;;
    esac
    printf '%s ' "$p"
  done
}

# Se senaliza pid a pid (y no con pkill -f) para respetar ese filtro.
matar() {
  local p
  for p in $(vivos); do kill "-$1" "$p" 2>/dev/null; done
}

# ---------------------------------------------------------------- argumentos
ACCION="iniciar"; MODO=""
for arg in "$@"; do
  case "$arg" in
    -h|--ayuda|--help)      ayuda; exit 0 ;;
    parar|cerrar|stop)      ACCION="parar" ;;
    estado|status)          ACCION="estado" ;;
    registro|log)           ACCION="registro" ;;
    -n|--sin-compilar)      COMPILAR=0 ;;
    -sv|--sin-ventana)      VENTANA=0 ;;
    -sh|--sin-hardware)     HARDWARE=0 ;;
    -b|--fondo)             FONDO=1 ;;
    rviz|gazebo|brazo)      MODO="$arg" ;;
    *) rojo "argumento desconocido: $arg"; echo; ayuda; exit 1 ;;
  esac
done
MODO="${MODO:-gazebo}"

# -------------------------------------------------------------------- ordenes
parar() {
  local pids
  pids=$(vivos)
  if [ -z "$pids" ]; then
    verde "No habia nada corriendo."
    rm -f "$PIDF"
    return 0
  fi
  neg "Cerrando la sesion (pids: $pids)"
  # Lo limpio es un SIGINT al grupo entero del lanzador: ros2 launch se lo pasa
  # a sus hijos y cada nodo cierra por su cuenta. El pkill de despues barre los
  # huerfanos y las sesiones que arranco otro (por ejemplo, a mano).
  if [ -f "$PIDF" ]; then
    kill -INT -- "-$(cat "$PIDF")" 2>/dev/null
  fi
  matar INT
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    [ -z "$(vivos)" ] && break
    sleep 1
  done
  if [ -n "$(vivos)" ]; then
    rojo "No se van con SIGINT; los mato: $(vivos)"
    matar KILL
    sleep 2
  fi
  rm -f "$PIDF"
  if [ -n "$(vivos)" ]; then
    rojo "No he podido cerrar: $(vivos)"
    rojo "Matalos a mano y vuelve a intentarlo."
    return 1
  fi
  verde "Todo cerrado."
}

# gz puede quedarse girando con el mundo vacio si el spawn le gana la carrera al
# servidor (contesta "Entity creation successful" y aun asi el modelo se queda
# sin plugins). Sin esta comprobacion el sintoma es un RViz con el brazo quieto
# y nadie diciendo por que.
salud_gazebo() {          # $1 = segundos que esperar al modelo (0 = una pasada)
  pgrep -f "gz-sim-main" >/dev/null 2>&1 || return 0    # modo rviz: nada que mirar
  local modelos margen=${1:-0} t0=$SECONDS
  while : ; do
    modelos=$(timeout 15 gz model --list 2>/dev/null | sed -n 's/^ *- *//p' \
              | grep -v '^ground_plane$' | tr '\n' ' ')
    [ -n "$modelos" ] && break
    [ $((SECONDS - t0)) -ge "$margen" ] && break
    sleep 2
  done
  if [ -z "$modelos" ]; then
    rojo "OJO: gazebo corre pero el mundo esta vacio (solo el suelo)."
    echo  "     El spawn le gano la carrera al servidor. Reinicia con mas margen:"
    echo  "         ./abrir.sh parar && ./abrir.sh -b"
    echo  "     y si se repite, sube el margen:  spawn_delay:=12"
    return 1
  fi
  if timeout 10 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
    verde "Robot en el mundo ($modelos) y publicando /joint_states."
    return 0
  fi
  rojo "OJO: el robot esta en el mundo ($modelos) pero /joint_states no llega."
  echo  "     Mira el puente:  ./abrir.sh registro"
  return 1
}

estado() {
  if [ -z "$(vivos)" ]; then
    rojo "SCARA parado: no hay ningun proceso."
    echo "Para arrancarlo:  ./abrir.sh"
    return 1
  fi
  verde "SCARA en marcha."
  echo
  neg "Procesos:"
  local p
  for p in $(vivos); do
    ps -o pid=,cmd= -p "$p" 2>/dev/null | cut -c1-100 | sed 's/^ */  /'
  done
  [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
  echo
  neg "Nodos ROS:"
  ros2 node list 2>/dev/null | sort | sed 's/^/  /'
  echo
  neg "Simulacion:"
  salud_gazebo
  local punta
  punta=$(timeout 5 ros2 topic echo /scara/esp_rec/linea --once 2>/dev/null \
          | sed -n 's/^data: //p')
  if [ -n "$punta" ]; then
    echo
    neg "La punta ahora mismo:"
    echo "  $punta"
  fi
}

registro() {
  if [ ! -f "$LOG" ]; then
    rojo "No hay registro en $LOG"
    echo "Solo se escribe si arrancas en segundo plano (./abrir.sh -b)."
    return 1
  fi
  neg "Siguiendo $LOG   (Ctrl+C para salir de aqui, la simulacion sigue)"
  tail -n 40 -f "$LOG"
}

# -------------------------------------------------------------------- entorno
source /opt/ros/lyrical/setup.bash
cd "$WS" || { rojo "no encuentro $WS"; exit 1; }

case "$ACCION" in
  parar)    parar;    exit $? ;;
  estado)   estado;   exit $? ;;
  registro) registro; exit $? ;;
esac

# ------------------------------------------------- ¿hay ya una sesion abierta?
# Dos launches a la vez son dos aduanas peleandose por el mismo /dev/ttyUSB0, y
# dos cc publicando en /joint_states.
VIEJOS=$(vivos)
if [ -n "$VIEJOS" ]; then
  rojo "Ya hay una simulacion abierta (pids: $VIEJOS)."
  printf "¿La cierro y sigo? [S/n] "
  read -r resp
  case "$resp" in
    [nN]*) echo "Abortado. Para cerrarla sin arrancar otra:  ./abrir.sh parar"; exit 1 ;;
  esac
  parar || exit 1
fi

# ------------------------------------------------------------------- compilar
if [ "$COMPILAR" -eq 1 ]; then
  neg "Compilando $PAQUETES ..."
  if ! colcon build --packages-select $PAQUETES; then
    rojo "La compilacion fallo. No lanzo nada."
    exit 1
  fi
fi
source install/setup.bash

# ---------------------------------------------------------------- que lanzar
case "$MODO" in
  gazebo) LANZADOR="gazebo.launch.py"; ARGS="rviz:=true" ;;
  rviz)   LANZADOR="rviz.launch.py";   ARGS="" ;;
  brazo)  LANZADOR="brazo.launch.py";  ARGS="" ;;
esac
[ "$VENTANA" -eq 0 ]  && ARGS="$ARGS ventana:=false"
[ "$HARDWARE" -eq 0 ] && ARGS="$ARGS hw:=false"

CFG="$WS/install/scara_kinematics/share/scara_kinematics/config/scara_urdf3.yaml"

echo
ETIQ="=== $MODO"
[ "$HARDWARE" -eq 0 ] && ETIQ="$ETIQ / sin placa"
neg "$ETIQ ==="
echo
echo "Cadena:  ESP32 -> esp (aduana) -> alvin | simon | teodoro -> cc -> esp_rec"
echo
if [ "$VENTANA" -eq 1 ]; then
  echo "Se abre la ventana del brazo: fija el cero, da potencia y prueba motores."
  echo "(sin ella:  ./abrir.sh -sv $MODO)"
  echo
fi
echo "Y desde otra terminal, con  cd ~/ros2_ws && source install/setup.bash :"
echo "    ros2 run scara_kinematics brazo_hw --ros-args --params-file $CFG"
echo "    ros2 topic echo /scara/esp_rec/linea     # donde esta la punta"
echo "    ros2 topic echo /scara/alvin/atascado    # cada motor se queja solo"
echo "    ros2 service call /scara/hw/zero std_srvs/srv/Trigger"
echo

# --------------------------------------------------------------------- lanzar
if [ "$FONDO" -eq 0 ]; then
  neg "Ctrl+C aqui para cerrarlo todo (o ./abrir.sh parar desde otra terminal)."
  echo
  exec ros2 launch scara_kinematics "$LANZADOR" $ARGS
fi

mkdir -p "$(dirname "$LOG")"
: > "$LOG"
# setsid: la simulacion queda en su propia sesion, asi sobrevive a que cierres
# la terminal y "parar" puede senalizar al grupo entero de una vez.
setsid nohup ros2 launch scara_kinematics "$LANZADOR" $ARGS >>"$LOG" 2>&1 &
echo $! > "$PIDF"
neg "Arrancando en segundo plano (pid $(cat "$PIDF"))."

# Los seis de brazo.launch.py, que va dentro de los tres modos. La aduana solo
# se lanza con placa (sin -sh).
ESPERADOS="/alvin /simon /teodoro /cc /esp_rec"
[ "$HARDWARE" -eq 1 ] && ESPERADOS="/esp $ESPERADOS"

printf 'Esperando a los nodos'
LISTO=0
for _ in $(seq 45); do
  NODOS=$(ros2 node list 2>/dev/null)
  FALTAN=""
  for n in $ESPERADOS; do
    grep -qx "$n" <<<"$NODOS" || FALTAN="$FALTAN $n"
  done
  if [ -z "$FALTAN" ]; then
    LISTO=1
    break
  fi
  printf '.'
  sleep 1
done
echo
if [ "$LISTO" -eq 1 ]; then
  verde "Nodos arriba."
  ros2 node list 2>/dev/null | sort | sed 's/^/  /'
  echo
  salud_gazebo 45          # el robot entra unos segundos despues que los nodos
else
  rojo "Faltan nodos:$FALTAN. Mira el log:  ./abrir.sh registro"
fi
echo
echo "    ./abrir.sh estado     ver que hay corriendo"
echo "    ./abrir.sh registro   seguir el log ($LOG)"
echo "    ./abrir.sh parar      cerrarlo todo"
