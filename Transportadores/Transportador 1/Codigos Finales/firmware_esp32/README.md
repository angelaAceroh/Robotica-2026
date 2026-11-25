# Firmware de la ESP32 (MicroPython)

| Archivo | Qué hace |
|---|---|
| `main.py` | se ejecuta al encender y arranca `control_vel.run()` |
| `control_vel.py` | **el control**: lee los encoders (PCNT), cierra el lazo de velocidad PI de J1–J3 cada 50 ms con prealimentación de rozamiento y gravedad, mueve las palas por cuentas, y aplica las protecciones |
| `pruebas/test_encoders.py` | comprueba las cuentas y el sentido de cada encoder moviendo las juntas a mano |
| `pruebas/gripper_prueba.py` | pruebas sueltas del motor N20 de las palas |

**Protecciones:**
- topes por software a 2° de cada límite;
- motores a 0 si no llega consigna en 500 ms;
- corte de una junta si se mueve al revés de lo pedido (lazo invertido) o a más de 300 °/s;
- las palas no obedecen hasta estar referenciadas.

**Subirlo:** ver el [manual de usuario](../../manual_de_usuario/README.md#3-subir-el-firmware-a-la-esp32-solo-si-se-cambia).
Diagrama de flujo: [`flujo_firmware`](../../documentacion/3_diagramas_de_flujo/flujo_firmware.png).
Pines: [plano eléctrico](../../documentacion/4_planos/electricos).
