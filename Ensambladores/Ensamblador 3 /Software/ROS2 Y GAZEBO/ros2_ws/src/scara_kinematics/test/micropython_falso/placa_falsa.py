"""Una ESP32 de mentira por un puerto serie: main.py de verdad, motores simulados.

    python3 src/scara_kinematics/test/micropython_falso/placa_falsa.py
    # dice: «puerto: /dev/pts/5»; en otra terminal:
    ros2 launch scara_kinematics efector.launch.py puerto:=/dev/pts/5 gz:=true

Sirve para probar toda la cadena (ventana -> vector -> USB -> main.py -> los
tres nodos) sin la placa. Lo que dice main.py sale tambien por esta terminal.
Con los pines y los 5500 pulsos por vuelta de verdad; main.py no se toca.

    FALLOS=alvin_inv python3 .../placa_falsa.py     # Alvin al reves de su encoder
    FALLOS=simon_b   python3 .../placa_falsa.py     # Simon sin canal B
    python3 .../placa_falsa.py otro_main.py          # con otro main.py (para probar cambios)
"""
import os
import pty
import select
import subprocess
import sys
import shutil
import tty

AQUI = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(AQUI, '..', '..', 'firmware', 'micropython', 'main.py')


def main():
    # Su "flash": empieza vacia cada vez (los tres motores en 0, como tras la 'h')
    carpeta = os.path.expanduser('~/.cache/scara_placa_falsa')
    shutil.rmtree(carpeta, ignore_errors=True)
    os.makedirs(carpeta)
    with open(sys.argv[1] if len(sys.argv) > 1 else MAIN) as f:
        codigo = f.read()
    with open(os.path.join(carpeta, 'main.py'), 'w') as f:
        f.write(codigo)

    placa, placa_tty = pty.openpty()
    tty.setraw(placa_tty)
    pc, pc_tty = pty.openpty()
    tty.setraw(pc_tty)
    print(f'puerto: {os.ttyname(pc_tty)}', flush=True)

    env = dict(os.environ, PYTHONPATH=AQUI)
    proc = subprocess.Popen([sys.executable, '-u', 'main.py'], cwd=carpeta,
                            stdin=placa_tty, stdout=placa_tty, stderr=subprocess.STDOUT,
                            env=env)
    try:
        while proc.poll() is None:
            listos, _, _ = select.select([placa, pc], [], [], 0.1)
            if placa in listos:
                datos = os.read(placa, 4096)
                os.write(pc, datos)
                sys.stdout.write(datos.decode('utf-8', 'replace'))
                sys.stdout.flush()
            if pc in listos:
                datos = os.read(pc, 4096)
                os.write(placa, datos)
                sys.stdout.write('>> ' + datos.decode('utf-8', 'replace'))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        proc.kill()


if __name__ == '__main__':
    main()
