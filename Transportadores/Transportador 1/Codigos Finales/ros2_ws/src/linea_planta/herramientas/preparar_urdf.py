#!/usr/bin/env python3
"""
Adapta los URDF de los otros cuatro equipos para que funcionen en ESTE Gazebo.

    python3 herramientas/preparar_urdf.py          (desde la raiz del paquete)

Lee   originales/*.urdf   (tal como los entrego cada equipo, sin tocar)
Escribe   urdf/<robot>.urdf

Por que hace falta:

  1. Los cuatro traen ros2_control con el plugin gz_ros2_control, que en este
     equipo NO esta instalado (por eso nuestro robot usa plugins nativos de
     gz-sim).  Se quita ros2_control y se pone, por cada articulacion movil, un
     JointPositionController nativo que escucha en
         /<robot>/<articulacion>/cmd_pos      (gz.msgs.Double, en rad o m)
     y un JointStatePublisher.  Es el mismo esquema que planta_brazo.

  2. El ensamblador 1 y el SCARA no tienen eslabon 'world': en Gazebo se
     caerian.  Se les anade 'world' y una junta fija a su eslabon raiz.

  3. El ensamblador 2 llega SIN mallas (el zip solo trae el URDF, que apunta a
     package://Ensamble/meshes/...).  Mientras su equipo no las pase, se dibuja
     con formas simples del tamano que dicen sus propias articulaciones.

  4. Se quitan las COLISIONES de los cuatro.  Son mallas de CAD pesadas (la
     LINK3 del ensamblador 1 pesa 2 MB) y, en una mesa con cinco brazos cuyos
     alcances se tocan, harian que se enganchasen entre si.  Para ver la linea
     y ensayar la coordinacion basta con las visuales; las inercias se dejan.

  5. Se anade un poco de amortiguamiento en cada articulacion: sus URDF no
     traen ninguno y, sin el, los controladores de posicion oscilan.

  6. Si una articulacion se llama IGUAL que un eslabon, se le anade '_joint'.
     En URDF es legal, pero Gazebo lo convierte a SDF, donde comparten nombres,
     y rechaza el modelo entero.  Le pasa al SCARA (ensamblador 3): sus
     articulaciones Simon, Teodoro y Dave mueven los eslabones del mismo
     nombre.  Sus temas pasan a ser /ensamblador3/Simon_joint/cmd_pos, etc.

  7. Si una articulacion tiene effort="0" en su <limit>, se le pone uno con el
     que se pueda mover.  En Gazebo el effort es el esfuerzo MAXIMO: con 0 el
     controlador calcula la fuerza que quiera y se recorta a cero, y la
     articulacion no se mueve nunca (en RViz no se nota, alli nada aplica
     fuerzas).  Le pasa al SCARA en sus tres articulaciones.  Y en las
     continuas se quitan lower/upper, que no significan nada (el SCARA trae
     lower=3 upper=0 y lower=6 upper=-3).

Lo que NO se toca: geometria, origenes de las articulaciones, ejes, masas e
inercias.  El robot es el que cada equipo diseno.
"""
import copy
import os
import sys
import xml.etree.ElementTree as ET

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#            nombre en la linea   archivo original
ROBOTS = [
    ('ensamblador1', 'ensamblador1_urdf_test_model.urdf'),
    ('ensamblador2', 'ensamblador2_Ensamble.urdf'),
    ('ensamblador3', 'ensamblador3_scara_urdf_2.urdf'),
    ('transportador2', 'transportador2_urdf_2.urdf'),
]

# Ganancias de los controladores de posicion.  NO son las de cada equipo (las
# suyas estaban en ros2_control, en archivos que no vinieron en los zip); son
# unas que sostienen los brazos quietos en la simulacion.
GANANCIAS = {
    'revolute':   dict(p=120.0, i=40.0, d=6.0, imax=20.0, cmax=60.0),
    'continuous': dict(p=120.0, i=40.0, d=6.0, imax=20.0, cmax=60.0),
    'prismatic':  dict(p=800.0, i=100.0, d=40.0, imax=50.0, cmax=200.0),
}
AMORTIGUAMIENTO = 0.5
ESFUERZO_SI_CERO = {'revolute': 50.0, 'continuous': 50.0, 'prismatic': 200.0}

# Formas para el ensamblador 2 (sin mallas).  Medidas sacadas de SUS
# articulaciones: J1 a 0.0647 de altura, J1->J2 = 0.20 m, J2->J3 = 0.127 m,
# J3 prismatica de 0 a 0.10 m.
FORMAS_E2 = {
    'base_link': ('cylinder', dict(radius=0.060, length=0.0647), (0, 0, 0.03235), (0.55, 0.55, 0.58)),
    'link_1':    ('box', dict(size='0.24 0.05 0.034'), (0.10, 0, 0.017), (0.55, 0.75, 0.95)),
    'link_2':    ('box', dict(size='0.17 0.045 0.026'), (0.0636, 0, 0.0128), (0.55, 0.75, 0.95)),
    'link_3':    ('cylinder', dict(radius=0.010, length=0.16), (0, 0, -0.08), (0.95, 0.95, 0.95)),
}


def eslabon_raiz(robot):
    hijos = {j.find('child').get('link') for j in robot.findall('joint')}
    raices = [l.get('name') for l in robot.findall('link') if l.get('name') not in hijos]
    if len(raices) != 1:
        raise ValueError('esperaba un solo eslabon raiz, hay %s' % raices)
    return raices[0]


def quitar_ros2_control(robot):
    n = 0
    for e in list(robot.findall('ros2_control')):
        robot.remove(e); n += 1
    for g in list(robot.findall('gazebo')):
        if any('ros2_control' in (p.get('filename') or '') for p in g.iter('plugin')):
            robot.remove(g); n += 1
    return n


def fijar_al_suelo(robot):
    raiz = eslabon_raiz(robot)
    if raiz == 'world':
        return False
    robot.insert(0, ET.Element('link', name='world'))
    j = ET.Element('joint', name='fijar_al_suelo', type='fixed')
    ET.SubElement(j, 'parent', link='world')
    ET.SubElement(j, 'child', link=raiz)
    ET.SubElement(j, 'origin', xyz='0 0 0', rpy='0 0 0')
    robot.insert(1, j)
    return True


def renombrar_juntas(robot):
    eslabones = {l.get('name') for l in robot.findall('link')}
    cambios = []
    for j in robot.findall('joint'):
        if j.get('name') in eslabones:
            nuevo = j.get('name') + '_joint'
            cambios.append((j.get('name'), nuevo))
            j.set('name', nuevo)
    return cambios


def arreglar_limites(robot):
    cambios = []
    for j in robot.findall('joint'):
        lim = j.find('limit')
        if lim is None or j.get('type') == 'fixed':
            continue
        if float(lim.get('effort', '0')) == 0.0:
            lim.set('effort', str(ESFUERZO_SI_CERO[j.get('type')]))
            cambios.append('%s: effort 0 -> %s' % (j.get('name'), lim.get('effort')))
        if j.get('type') == 'continuous' and ('lower' in lim.attrib or 'upper' in lim.attrib):
            cambios.append('%s: quitados lower=%s upper=%s (continua)'
                           % (j.get('name'), lim.attrib.pop('lower', None), lim.attrib.pop('upper', None)))
    return cambios


def quitar_colisiones(robot):
    n = 0
    for l in robot.findall('link'):
        for c in list(l.findall('collision')):
            l.remove(c); n += 1
    return n


def poner_formas(robot, formas):
    for l in robot.findall('link'):
        if l.get('name') not in formas:
            continue
        tipo, attrs, xyz, rgb = formas[l.get('name')]
        for v in list(l.findall('visual')):
            l.remove(v)
        v = ET.SubElement(l, 'visual')
        ET.SubElement(v, 'origin', xyz='%g %g %g' % xyz, rpy='0 0 0')
        geo = ET.SubElement(v, 'geometry')
        ET.SubElement(geo, tipo, **{k: str(val) for k, val in attrs.items()})
        mat = ET.SubElement(v, 'material', name='forma_' + l.get('name'))
        ET.SubElement(mat, 'color', rgba='%g %g %g 1' % rgb)


def amortiguar(robot):
    for j in robot.findall('joint'):
        if j.get('type') == 'fixed':
            continue
        if j.find('dynamics') is None:
            ET.SubElement(j, 'dynamics', damping=str(AMORTIGUAMIENTO), friction='0.0')


def anadir_control(robot, nombre):
    juntas = [j for j in robot.findall('joint') if j.get('type') != 'fixed']
    for j in juntas:
        g = GANANCIAS[j.get('type')]
        gz = ET.SubElement(robot, 'gazebo')
        p = ET.SubElement(gz, 'plugin', filename='gz-sim-joint-position-controller-system',
                          name='gz::sim::systems::JointPositionController')
        for tag, val in (('joint_name', j.get('name')),
                         ('topic', '/%s/%s/cmd_pos' % (nombre, j.get('name'))),
                         ('p_gain', g['p']), ('i_gain', g['i']), ('d_gain', g['d']),
                         ('i_max', g['imax']), ('i_min', -g['imax']),
                         ('cmd_max', g['cmax']), ('cmd_min', -g['cmax'])):
            ET.SubElement(p, tag).text = str(val)
    gz = ET.SubElement(robot, 'gazebo')
    ET.SubElement(gz, 'plugin', filename='gz-sim-joint-state-publisher-system',
                  name='gz::sim::systems::JointStatePublisher')
    return [j.get('name') for j in juntas]


def preparar(nombre, archivo):
    arbol = ET.parse(os.path.join(RAIZ, 'originales', archivo))
    robot = arbol.getroot()
    robot.set('name', nombre)
    quitados = quitar_ros2_control(robot)
    fijado = fijar_al_suelo(robot)
    renombradas = renombrar_juntas(robot)
    limites = arreglar_limites(robot)
    if nombre == 'ensamblador2':
        poner_formas(robot, FORMAS_E2)
    colisiones = quitar_colisiones(robot)
    amortiguar(robot)
    juntas = anadir_control(robot, nombre)
    cabecera = ET.Comment(
        ' GENERADO por herramientas/preparar_urdf.py a partir de originales/%s. '
        'No editar a mano: editar el original o la herramienta y regenerar. ' % archivo)
    robot.insert(0, cabecera)
    ET.indent(arbol, space='  ')
    salida = os.path.join(RAIZ, 'urdf', nombre + '.urdf')
    arbol.write(salida, encoding='utf-8', xml_declaration=True)
    print('%-15s ros2_control quitado: %d bloques | fijado al suelo: %s | '
          'colisiones quitadas: %d | articulaciones: %s'
          % (nombre, quitados, 'si' if fijado else 'ya lo estaba', colisiones,
             ', '.join(juntas)))
    for viejo, nuevo in renombradas:
        print('    articulacion renombrada: %s -> %s  (se llamaba igual que un eslabon)'
              % (viejo, nuevo))
    for c in limites:
        print('    limite corregido: %s' % c)


if __name__ == '__main__':
    for nombre, archivo in ROBOTS:
        preparar(nombre, archivo)
    sys.exit(0)
