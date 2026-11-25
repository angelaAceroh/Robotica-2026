# Hojas de especificaciones de los dispositivos

Datos de los componentes que usa el Transportador 1. Donde el fabricante publica una hoja
de datos, se enlaza. Los motorreductores son genéricos de tienda: sus datos son los de la
ficha del vendedor, y los que se **midieron en el robot** van marcados.

## Resumen

| Dispositivo | Cant. | Función | Datos clave | Hoja de datos |
|---|---|---|---|---|
| ESP32 DevKit (ESP32-WROOM-32), 38 pines | 1 | microcontrolador: lazos de velocidad, encoders, gripper | 240 MHz, 3,3 V lógicos, 4 contadores de pulsos (PCNT) usados, USB-serie | en esta carpeta: [`esp32_datasheet_en.pdf`](esp32_datasheet_en.pdf) y [`esp32-wroom-32_datasheet_en.pdf`](esp32-wroom-32_datasheet_en.pdf) (Espressif) |
| Driver L298N (módulo) | 2 | puente H doble: PWM y sentido de los motores | hasta 46 V, 2 A por canal, caída de unos 2 V | [STMicroelectronics L298](https://www.st.com/resource/en/datasheet/l298.pdf) |
| Motorreductor TS-25GAH370-45 (12 V) con encoder Hall | 2 | J1 (base) y J3 (codo) | caja 45:1, 130 rpm en vacío, 2,5 W, 0,15 A en vacío, **par nominal 0,1157 N·m** (2,6 lb·cm) | ficha del vendedor (datos aquí) |
| Motorreductor 12 V, 60 rpm, con encoder Hall | 1 | J2 (hombro) | caja **98:1** (medida: 97,0:1), 0,65 N·m en la junta tras la correa | ficha del vendedor (datos aquí) |
| Encoder Hall TSINY (en los motorreductores) | 3 | posición y sentido de J1–J3 | 12 pulsos/vuelta del motor por canal, A/B en cuadratura, Vcc 5–24 V según la hoja (aquí, 3,3 V) | hoja TSINY-8370 incluida con los motores |
| Micromotor N20 con encoder (CH-N20-3) | 1 | J4: palas del gripper (piñón-cremallera) | encoder A/B a 3,3 V; carrera útil 0–580 cuentas; ~770 cuentas/s al 25 % | ficha del vendedor (datos aquí) |
| Correa 200-2GT + poleas GT2 20 T y 60 T (eje 8 mm) | 3 juegos | segunda reducción de J1–J3 | paso 2 mm, 100 dientes, relación 3:1 | — |
| Rodamientos 628-2RS (8×24×8 mm) | 8 | apoyos de los ejes | — | (de catálogo) |
| Rodamiento axial 51306 | 1 | apoyo axial de la base | — | (de catálogo) |
| Fuente 12 V DC | 1 | potencia de los L298N | tierra común con la ESP32 | — |

## Notas de montaje que vienen de las hojas de datos

- **Niveles lógicos.** Las GPIO de la ESP32 son de 3,3 V y no toleran 5 V. Por eso los
  encoders se alimentan a 3,3 V. Si se alimentaran a 5 V, harían falta divisores de
  10 kΩ / 20 kΩ en cada señal.
- **GPIO 34, 35, 36 y 39 son solo de entrada** y no tienen pull-up interno. Las 34 y 35
  llevan pull-up externo de 10 kΩ a 3,3 V.
- **Pines de arranque (strapping).** Las GPIO 0, 2, 5, 12 y 15 deciden cómo arranca la
  placa. La GPIO 2 se usa a propósito como ENB del gripper: su pull-down interno durante el
  reset deja el canal **deshabilitado por hardware** al encender.
- **L298N.** Hay que quitar los jumpers de ENA y ENB, porque con ellos el enable queda fijo
  y el PWM no actúa. El freno es con IN1 = IN2 = 1 y el enable activo; la rueda libre, con
  el enable a 0.
- **Par de bloqueo.** La ficha del TS-25GAH370 no lo trae. Se estima en 2,75 × el nominal y
  está pendiente de medir.

El pinout completo está en `../4_planos/electricos/plano_electrico.png` y en
`../4_planos/electricos/conexiones_y_montaje.md`.
