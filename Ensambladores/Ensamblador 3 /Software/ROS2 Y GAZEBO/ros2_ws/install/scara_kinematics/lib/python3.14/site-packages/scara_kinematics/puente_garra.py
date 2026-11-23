"""Nodo `puente_garra` (se llama `puente_placa`, como el de siempre): puente_placa
con las ordenes de la garra (GARRA.md). Lo lanza traycho_garra.launch.py.

puente_placa no se ha tocado: este lo HEREDA entero (la V, la telemetria, los
vigilantes, la 'q' al cerrar) y solo deja pasar a la placa dos ordenes mas por
scara/placa/orden:

    1 | abrir      '1' a la placa: garra.py abre (10 grados)
    0 | cerrar     '0' a la placa: garra.py cierra (50, «chocolate»)

La placa contesta «GARRA abierta 10» o «GARRA cerrada 50»; sale por /rosout como
«placa: GARRA ...», igual que las demas lineas, y de ahi la leen la ventana y el
traycho con garra (garra_traycho).
"""
from __future__ import annotations

from std_msgs.msg import String

from scara_kinematics import puente_placa as PP

ORDENES = {'1': '1', 'abrir': '1', '0': '0', 'cerrar': '0'}


class PuenteGarra(PP.PuentePlaca):

    def on_orden(self, msg: String):
        palabras = msg.data.split()
        if palabras and palabras[0].lower() in ORDENES:
            self._mandar(ORDENES[palabras[0].lower()])
            return
        super().on_orden(msg)


def main(args=None):
    # El main de puente_placa (con su 'q' al cerrar), con este nodo
    PP.PuentePlaca = PuenteGarra
    PP.main(args)


if __name__ == '__main__':
    main()
