#!/usr/bin/env python3
"""
JACOBIANO del robot transportador 3R.  Modulo independiente.

Antes esto estaba escondido en dos sitios: la matriz se construia dentro de
cinematica.py y se invertia a mano en mover_robot_vel.py con un
np.linalg.pinv() suelto en medio del lazo de control.  Aqui esta todo junto y
con nombre propio.

------------------------------------------------------------------- que es
El jacobiano relaciona VELOCIDADES, no posiciones.  Si p(q) es la posicion del
extremo que da la cinematica directa, el jacobiano es su derivada respecto a
las tres juntas:

                | dx/dth1   dx/dth2   dx/dth3 |
    J(q)  =     | dy/dth1   dy/dth2   dy/dth3 |        (3x3, en metros/radian)
                | dz/dth1   dz/dth2   dz/dth3 |

y con eso:

    DIRECTO     x_punto = J(q) * q_punto      velocidad del TCP a partir de
                                              las velocidades de juntura
    INVERSO     q_punto = J(q)^-1 * x_punto   lo que necesita el control:
                                              "quiero que la punta vaya a esta
                                              velocidad, ¿que velocidad le pido
                                              a cada motor?"

Se usa la PSEUDOINVERSA (np.linalg.pinv) y no la inversa normal: cerca de una
singularidad el determinante se va a cero, la inversa explota y pediria
velocidades infinitas; la pseudoinversa degrada en vez de reventar.

------------------------------------------------------------- singularidades
Para este brazo el determinante sale (con r = a1 + a2*c2 + a3*c23):

    det J = -r * a2 * a3 * sin(th3)

o sea que hay dos formas de perder un grado de libertad:

  1) sin(th3) = 0   ->  th3 = 0 o th3 = +-180 grados.  Brazo ESTIRADO o
     completamente PLEGADO: hombro y codo tiran en la misma linea, el extremo
     ya no puede moverse en la direccion radial.
  2) r = 0          ->  el extremo cae sobre el eje de la base.  Girar la base
     no mueve el TCP, asi que th1 no sirve para nada ahi.

Este modulo las detecta y las avisa en vez de dejar que el control pida
velocidades absurdas.

--------------------------------------------------------------------- uso
    import jacobiano as jac

    J   = jac.jacobiano(q)             # 3x3
    Ji  = jac.inverso(q)               # pseudoinversa 3x3
    xd  = jac.vel_tcp(q, qd)           # juntas -> TCP   (m/s)
    qd  = jac.vel_juntas(q, xd)        # TCP -> juntas   (rad/s)
    jac.diagnostico(q)                 # det, condicion, singularidad

    python3 jacobiano.py               # lo imprime simbolico y lo comprueba
"""
import math

import numpy as np
import sympy as sp

import config as cfg

# margen por debajo del cual se considera que el brazo esta en singularidad
DET_MIN = 1e-6          # m^3/rad^3
COND_MAX = 1e4          # numero de condicion a partir del cual avisa


# --------------------------------------------------------------- construccion
def _simbolico():
    """Construye T, la posicion y el jacobiano en simbolico (una sola vez).

    Devuelve (pos, J, simbolos).  Es la MISMA cadena DH que cinematica.py:
    de hecho cinematica.py importa de aqui, para que no haya dos copias de
    las matrices que puedan quedar descuadradas.
    """
    th1, th2, th3, L1, a1, a2, a3 = sp.symbols('th1 th2 th3 L1 a1 a2 a3',
                                               real=True)

    A01 = sp.Matrix([
        [sp.cos(th1), 0,  sp.sin(th1), a1 * sp.cos(th1)],
        [sp.sin(th1), 0, -sp.cos(th1), a1 * sp.sin(th1)],
        [0,           1,  0,           L1],
        [0,           0,  0,           1]])

    A12 = sp.Matrix([
        [sp.cos(th2), -sp.sin(th2), 0, a2 * sp.cos(th2)],
        [sp.sin(th2),  sp.cos(th2), 0, a2 * sp.sin(th2)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])

    A23 = sp.Matrix([
        [sp.cos(th3), -sp.sin(th3), 0, a3 * sp.cos(th3)],
        [sp.sin(th3),  sp.cos(th3), 0, a3 * sp.sin(th3)],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])

    T = sp.simplify(A01 * A12 * A23)
    pos = T[0:3, 3]
    J = pos.jacobian([th1, th2, th3])        # <-- EL JACOBIANO
    return pos, J, (th1, th2, th3, L1, a1, a2, a3)


_POS_SIM, _JAC_SIM, _SIM = _simbolico()

# compilados: evaluar estos tarda microsegundos, el .subs() simbolico tardaba
# milisegundos y se comia el presupuesto del lazo de 50 ms
POS_COMPILADA = sp.lambdify(_SIM, _POS_SIM, 'numpy')
JAC_COMPILADA = sp.lambdify(_SIM, _JAC_SIM, 'numpy')

GEO = (cfg.L1, cfg.A1, cfg.A2, cfg.A3)


# ------------------------------------------------------------------ numerico
def jacobiano(q):
    """q en radianes (3,) -> jacobiano de velocidad lineal, 3x3 [m/rad]."""
    return np.asarray(JAC_COMPILADA(q[0], q[1], q[2], *GEO), dtype=float)


def determinante(q):
    """det J(q).  Cero exacto = singularidad."""
    return float(np.linalg.det(jacobiano(q)))


def det_analitico(q):
    """El mismo determinante por la formula cerrada: -r*a2*a3*sin(th3).

    Esta aqui para comprobar que la formula del informe coincide con el
    calculo numerico, y porque leyendola se entiende DE DONDE sale la
    singularidad.
    """
    _, th2, th3 = q[0], q[1], q[2]
    L1, a1, a2, a3 = GEO
    r = a1 + a2 * math.cos(th2) + a3 * math.cos(th2 + th3)
    return -r * a2 * a3 * math.sin(th3)


def inverso(q, amortiguacion=0.0):
    """Pseudoinversa del jacobiano, 3x3 [rad/m].

    amortiguacion > 0 usa minimos cuadrados amortiguados (Levenberg):

        J# = J^T (J J^T + lambda^2 I)^-1

    que suaviza el paso por singularidades a cambio de un poco de error de
    seguimiento.  Con amortiguacion = 0 es la pseudoinversa de Moore-Penrose
    tal cual, que es lo que usa mover_robot_vel.py.
    """
    J = jacobiano(q)
    if amortiguacion <= 0.0:
        return np.linalg.pinv(J)
    lam2 = float(amortiguacion) ** 2
    return J.T @ np.linalg.inv(J @ J.T + lam2 * np.eye(3))


# ------------------------------------------------------------ los dos sentidos
def vel_tcp(q, qd):
    """Velocidades de juntura [rad/s] -> velocidad del TCP [m/s].

        x_punto = J(q) * q_punto
    """
    return jacobiano(q) @ np.asarray(qd, dtype=float).reshape(3)


def vel_juntas(q, xd, amortiguacion=0.0):
    """Velocidad del TCP [m/s] -> velocidades de juntura [rad/s].

        q_punto = pinv(J(q)) * x_punto

    Es la linea que gobierna el robot cuando se manda la trayectoria en
    cartesiano (mover_robot_vel.py, modo --cartesiano).
    """
    return inverso(q, amortiguacion) @ np.asarray(xd, dtype=float).reshape(3)


def vel_juntas_grados(q_deg, xd, amortiguacion=0.0):
    """Igual que vel_juntas pero con grados a la entrada y a la salida.

    El firmware de la ESP32 habla en grados/s, asi que esta es la que evita
    el error de unidades que ya nos costo un susto (meter grados/s en un
    controlador disenado en rad/s multiplica la ganancia por 57)."""
    q = np.radians(np.asarray(q_deg, dtype=float))
    return np.degrees(vel_juntas(q, xd, amortiguacion))


# ------------------------------------------------------------- singularidades
def singular(q, det_min=DET_MIN):
    """True si la postura esta en (o pegada a) una singularidad."""
    return abs(determinante(q)) < det_min


def motivo(q, det_min=DET_MIN):
    """Explica POR QUE es singular, o cadena vacia si no lo es."""
    _, th2, th3 = q[0], q[1], q[2]
    L1, a1, a2, a3 = GEO
    r = a1 + a2 * math.cos(th2) + a3 * math.cos(th2 + th3)
    razones = []
    if abs(math.sin(th3)) < 1e-4:
        razones.append('codo alineado: th3 = %.1f grados (brazo %s)'
                       % (math.degrees(th3),
                          'estirado' if abs(math.cos(th3) - 1) < 1e-3
                          else 'plegado'))
    if abs(r) < 1e-4:
        razones.append('extremo sobre el eje de la base: r = %.4f m' % r)
    if not razones and abs(determinante(q)) < det_min:
        razones.append('determinante casi nulo: %.3e' % determinante(q))
    return '; '.join(razones)


def diagnostico(q):
    """Dict con todo lo que hace falta para juzgar una postura."""
    J = jacobiano(q)
    d = float(np.linalg.det(J))
    s = np.linalg.svd(J, compute_uv=False)
    return {
        'J': J,
        'det': d,
        'det_analitico': det_analitico(q),
        'singulares': s,                              # valores singulares
        'condicion': float(s[0] / s[-1]) if s[-1] > 1e-15 else float('inf'),
        'manipulabilidad': float(abs(d)),             # medida de Yoshikawa
        'singular': abs(d) < DET_MIN,
        'mal_condicionado': (s[-1] <= 1e-15
                             or s[0] / s[-1] > COND_MAX),
        'motivo': motivo(q),
    }


def imprimir_simbolico():
    """Saca por pantalla el jacobiano termino a termino, como en el informe."""
    th1, th2, th3, L1, a1, a2, a3 = _SIM
    print('J(q) simbolico, columna j = derivada respecto a th_j:\n')
    nombres = ('x', 'y', 'z')
    for i in range(3):
        for j in range(3):
            t = sp.simplify(sp.trigsimp(_JAC_SIM[i, j]))
            print('  d%s/dth%d = %s' % (nombres[i], j + 1, t))
        print()
    det = sp.simplify(sp.trigsimp(_JAC_SIM.det()))
    print('det J = %s' % sp.factor(det))
    print('\n(comprobacion: equivale a  -r*a2*a3*sin(th3)  con '
          'r = a1 + a2*cos(th2) + a3*cos(th2+th3))')


# ------------------------------------------------------------------- pruebas
def _tabla(M, fmt='%10.5f'):
    return '\n'.join('    [' + ' '.join(fmt % v for v in fila) + ' ]'
                     for fila in M)


if __name__ == '__main__':
    import random
    import time

    print('=' * 70)
    print('JACOBIANO DEL PLANTA BRAZO 3R')
    print('=' * 70)
    print('geometria: L1=%.3f  a1=%.3f  a2=%.3f  a3=%.3f  m\n' % GEO)

    imprimir_simbolico()

    # ---- evaluado en la pose de arranque
    q0 = np.radians(cfg.POSE_INICIAL)
    print('\n' + '=' * 70)
    print('EVALUADO EN LA POSE DE ARRANQUE  q = %s grados'
          % (tuple(cfg.POSE_INICIAL),))
    print('=' * 70)
    p0 = np.asarray(POS_COMPILADA(q0[0], q0[1], q0[2], *GEO),
                    dtype=float).reshape(3)
    print('  TCP = (%.1f, %.1f, %.1f) mm\n' % tuple(p0 * 1000))
    d = diagnostico(q0)
    print('J  [m/rad]:')
    print(_tabla(d['J']))
    print('\npinv(J)  [rad/m]:')
    print(_tabla(inverso(q0)))
    print('\n  det J          = %.6f   (formula cerrada %.6f)'
          % (d['det'], d['det_analitico']))
    print('  valores sing.  = %s' % np.array2string(d['singulares'],
                                                    precision=5))
    print('  n. condicion   = %.1f' % d['condicion'])
    print('  singular       = %s %s' % (d['singular'],
                                        '(' + d['motivo'] + ')'
                                        if d['motivo'] else ''))

    # ---- ejemplo con numeros, de los dos sentidos
    print('\n' + '=' * 70)
    print('EJEMPLO: quiero que la punta se mueva 50 mm/s en +Y')
    print('=' * 70)
    xd = np.array([0.0, 0.05, 0.0])
    qd = vel_juntas(q0, xd)
    print('  x_punto pedido  = (%.3f, %.3f, %.3f) m/s' % tuple(xd))
    print('  q_punto = pinv(J)*x_punto = (%.2f, %.2f, %.2f) grados/s'
          % tuple(np.degrees(qd)))
    print('  -> solo gira la base, que es lo que tiene que pasar: en esa')
    print('     postura el brazo esta en el plano XZ y +Y es tangencial.')
    comprob = vel_tcp(q0, qd)
    print('  vuelta atras J*q_punto = (%.4f, %.4f, %.4f) m/s  (error %.2e)'
          % (comprob[0], comprob[1], comprob[2],
             float(np.linalg.norm(comprob - xd))))

    # ---- singularidades
    print('\n' + '=' * 70)
    print('SINGULARIDADES')
    print('=' * 70)
    casos = (('brazo estirado   th3 =    0', (0.0, 0.3, 0.0)),
             ('brazo plegado    th3 = -180', (0.0, 0.3, -math.pi)),
             ('casi estirado    th3 =   -2', (0.0, 0.3, math.radians(-2))),
             ('postura normal   th3 =  -90', (0.0, 0.3, math.radians(-90))))
    print('  %-28s %12s %10s  %s'
          % ('caso', 'det J', 'condicion', 'aviso'))
    for nombre, q in casos:
        d = diagnostico(q)
        print('  %-28s %12.3e %10.1f  %s'
              % (nombre, d['det'], d['condicion'], d['motivo'] or '-'))

    # ---- coherencia contra diferencias finitas
    print('\n' + '=' * 70)
    print('COMPROBACION: J contra diferencias finitas, 2000 posturas')
    print('=' * 70)
    h = 1e-7
    peor = 0.0
    for _ in range(2000):
        q = np.array([random.uniform(-2.0, 2.0),
                      random.uniform(-1.4, 1.4),
                      random.uniform(-2.5, 2.5)])
        J = jacobiano(q)
        Jn = np.zeros((3, 3))
        for j in range(3):
            dq = np.zeros(3); dq[j] = h
            pa = np.asarray(POS_COMPILADA(*(q + dq), *GEO),
                            dtype=float).reshape(3)
            pb = np.asarray(POS_COMPILADA(*(q - dq), *GEO),
                            dtype=float).reshape(3)
            Jn[:, j] = (pa - pb) / (2 * h)
        peor = max(peor, float(np.max(np.abs(J - Jn))))
    print('  peor diferencia: %.3e m/rad' % peor)

    print('\nCOMPROBACION: pinv(J)*J = I fuera de singularidades')
    peor = 0.0
    for _ in range(2000):
        q = np.array([random.uniform(-2.0, 2.0),
                      random.uniform(-1.4, 1.4),
                      random.choice([1.0, -1.0]) * random.uniform(0.3, 2.5)])
        if singular(q):
            continue
        E = inverso(q) @ jacobiano(q) - np.eye(3)
        peor = max(peor, float(np.max(np.abs(E))))
    print('  peor desvio de la identidad: %.3e' % peor)

    t0 = time.perf_counter()
    for _ in range(5000):
        jacobiano([0.1, 0.2, 0.3])
    t_j = (time.perf_counter() - t0) / 5000
    t0 = time.perf_counter()
    for _ in range(5000):
        inverso([0.1, 0.2, 0.3])
    t_i = (time.perf_counter() - t0) / 5000
    print('\ncoste por evaluacion:  J %.1f us   pinv(J) %.1f us'
          % (t_j * 1e6, t_i * 1e6))
    print('  en el lazo de %.0f ms: %.2f %% del presupuesto'
          % (cfg.PERIODO * 1000, t_i / cfg.PERIODO * 100))
