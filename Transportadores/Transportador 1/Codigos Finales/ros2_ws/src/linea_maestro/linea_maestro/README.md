# linea_maestro · código

| Archivo | Qué es |
|---|---|
| `maestro.py` | **nodo maestro**: espera a los 5 robots, recorre la secuencia de 9 pasos (orden → `ocupado` → `listo`, con reenvío cada 1 s), vigila errores, silencios de más de 3 s y pasos de más de 60 s, y publica `/parada` |
| `robot_simulado.py` | se hace pasar por cualquiera de los 5 robots (parámetros `nombre` y `duracion`) para probar el maestro sin los robots reales |

Diagramas: [flujo de la línea completa](../../../../../documentacion/3_diagramas_de_flujo/flujo_maestro_linea_completa.png) ·
[secuencia del ciclo](../../../../../documentacion/2_diagramas_de_secuencia/secuencia_ciclo_linea.png)
