"""FK "de referencia" recorriendo la cadena completa del URDF.

No sustituye al modelo de model.py (que es el que usan los nodos, en forma
cerrada); sirve para comprobarlo: si el URDF cambia, el test compara las dos
cinematicas y falla si se separan mas de la tolerancia.

Solo depende de la libreria estandar: nada de numpy ni de KDL.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

Mat = list[list[float]]


def _matmul(a: Mat, b: Mat) -> Mat:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _rpy(r: float, p: float, y: float) -> Mat:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp, cp * sr, cp * cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _translate(m: Mat, xyz) -> Mat:
    t = [[1.0, 0, 0, xyz[0]], [0, 1.0, 0, xyz[1]], [0, 0, 1.0, xyz[2]], [0, 0, 0, 1.0]]
    return _matmul(m, t)


def _axis_rot(axis, q: float) -> Mat:
    n = math.sqrt(sum(v * v for v in axis)) or 1.0
    x, y, z = (v / n for v in axis)
    c, s, t = math.cos(q), math.sin(q), 1.0 - math.cos(q)
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


class UrdfChain:
    """Cadena serie leida de un URDF, en el orden en que aparecen los <joint>."""

    def __init__(self, path: str):
        root = ET.parse(path).getroot()
        self.joints = []
        for j in root.findall('joint'):
            origin = j.find('origin')
            axis = j.find('axis')
            xyz = (origin.get('xyz') if origin is not None else None) or '0 0 0'
            rpy = (origin.get('rpy') if origin is not None else None) or '0 0 0'
            ax = (axis.get('xyz') if axis is not None else None) or '1 0 0'
            self.joints.append({
                'name': j.get('name'),
                'type': j.get('type'),
                'parent': j.find('parent').get('link'),
                'child': j.find('child').get('link'),
                'xyz': [float(v) for v in xyz.split()],
                'rpy': [float(v) for v in rpy.split()],
                'axis': [float(v) for v in ax.split()],
            })
        # Los <joint> no vienen ordenados en el fichero (el fijo al mundo suele
        # ir al final), asi que reconstruimos la cadena desde la raiz: el unico
        # link que es padre y nunca hijo.
        children = {j['child'] for j in self.joints}
        roots = [j['parent'] for j in self.joints if j['parent'] not in children]
        self.root = roots[0] if roots else self.joints[0]['parent']
        by_parent = {}
        for j in self.joints:
            by_parent.setdefault(j['parent'], []).append(j)
        ordered, stack = [], [self.root]
        while stack:
            for j in by_parent.get(stack.pop(), []):
                ordered.append(j)
                stack.append(j['child'])
        self.joints = ordered

    def frames(self, q: dict[str, float]) -> dict[str, Mat]:
        """Transformadas de cada link respecto de la raiz."""
        m = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        out = {}
        frames_by_link = {self.root: [row[:] for row in m]}
        for j in self.joints:
            # <origin> del URDF = trasladar xyz y luego rotar rpy, sobre el padre
            m = _translate(frames_by_link[j['parent']], j['xyz'])
            m = _matmul(m, _rpy(*j['rpy']))
            v = q.get(j['name'], 0.0)
            if j['type'] in ('revolute', 'continuous'):
                m = _matmul(m, _axis_rot(j['axis'], v))
            elif j['type'] == 'prismatic':
                n = math.sqrt(sum(a * a for a in j['axis'])) or 1.0
                m = _translate(m, [a / n * v for a in j['axis']])
            frames_by_link[j['child']] = [row[:] for row in m]
            out[j['child']] = [row[:] for row in m]
        return out
