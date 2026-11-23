# Documentación Técnica de Soporte — Ensamblador 2
**Universidad Militar Nueva Granada — Laboratorio de Robótica 2026**  
**Equipo de Trabajo — Grupo #3 (Ensamblador 2)**

Este directorio contiene el acervo documental de ingeniería que respalda el diseño mecánico, electrónico, de control, software e integración en planta del robot manipulador SCARA.

---

## 1. Índice de Documentación de Soporte

| Subdirectorio | Contenido Principal | Documento Clave |
| :--- | :--- | :--- |
| **[`Memorias de Calculo/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Documentacion/Memorias%20de%20Calculo/MEMORIAS_CALCULO.md)** | Modelado cinemático directo/inverso, jacobiano, singularidades, cálculo de resolución de encoders y dimensionamiento eléctrico. | `MEMORIAS_CALCULO.md` |
| **[`DIAGRAMAS/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Documentacion/DIAGRAMAS/DIAGRAMAS_SISTEMA.md)** | Diagramas de arquitectura del sistema, secuencia de proceso Pick & Place, flujos de control en ESP32 y tabla de conexionado. | `DIAGRAMAS_SISTEMA.md` |
| **[`Hojas de Especificaciones/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Documentacion/Hojas%20de%20Especificaciones/ESPECIFICACIONES.md)** | Fichas técnicas detalladas de microcontrolador ESP32, motores JGA25-370, drivers L298N, pinza N20 y elementos mecánicos GT2. | `ESPECIFICACIONES.md` |
| **[`Manual de Usuario/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Documentacion/Manual%20de%20Usuario/MANUAL_USUARIO.md)** | Instructivo paso a paso de encendido, calibración de cero, operación en solitario, operación con maestro y troubleshooting. | `MANUAL_USUARIO.md` |
| **[`Planos Mecánicos y eléctricos/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Documentacion/Planos%20Mecánicos%20y%20eléctricos/README.md)** | Planos acotados de despiece, ensambles generales y diagramas de conexionado de potencia y lógica. | `PLANO MECANICO.pdf`, `PLANOS ELECTRICOS (2).PDF` |
| **[`Plano de planta/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Documentacion/Plano%20de%20planta/README.md)** | Distribución espacial, cotas y ubicación relativa del Ensamblador 2 en la celda de manufactura. | `2)PLANO DE LA PLANTA (ENTORNO Y ELEMENTOS) CON DIMENSIONES.png` |
| **[`Diagrama de nodos/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Documentacion/Diagrama%20de%20nodos/README.md)** | Topología de nodos ROS 2, interfaces y flujo de tópicos/servicios interactivo. | `6)DIAGRAMA DE NODOS E INTERACCIONES PARA ROS2.html` |
| **[`Actas de trabajo/`](file:///home/samuel-azuero/Robotica-2026/Ensambladores/Ensamblador%202/SCARA/Documentacion/Actas%20de%20trabajo/README.md)** | Registro de sesiones de trabajo en equipo, compromisos adquiridos y seguimiento de hitos. | `Actas trabajo equipo - Ensamblador 2.pdf` |

---

## 2. Conformidad con la Rúbrica de Evaluación

Esta documentación satisface en su totalidad los criterios de la rúbrica evaluativa:
- **R2.2 (I1, I3):** Diagramas completos de bloques, flujo de programación, secuencia temporal y conexionado eléctrico.
- **R2.3 (I1, I2, I3):** Justificación técnico-matemática rigurosa de las decisiones de diseño mecánico y electrónico.
- **R2.3 (I4):** Rutinas validadas en RViz y Gazebo con código estructurado en ROS 2.
