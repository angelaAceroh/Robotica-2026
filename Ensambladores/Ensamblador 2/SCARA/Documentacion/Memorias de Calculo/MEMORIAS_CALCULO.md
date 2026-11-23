# Memorias de Cálculo — Robot Manipulador SCARA RRP (Ensamblador 2)
**Universidad Militar Nueva Granada — Laboratorio de Robótica**  
**Equipo de Trabajo — Grupo #3 (Ensamblador 2)**  
* Samuel David Azuero Forero
* Dilhann Adrián Yañez Nieves
* Ana María Iglesias Roa
* Julian Camilo Paez Colmenares

---

## 1. Introducción y Parámetros Estructurales

El manipulador robótico corresponde a una arquitectura **SCARA (Selective Compliance Assembly Robot Arm)** de configuración cinemática **RRP** (Rotacional - Rotacional - Prismático), complementada por un efector final tipo pinza paralela (Gripper) accionado por un micromotor N20 con husillo.

### Parámetros Geométricos del Manipulador
| Parámetro | Símbolo | Valor Numérico | Descripción |
| :--- | :--- | :--- | :--- |
| **Longitud Eslabón 1** | $L_1$ | $0.205\text{ m}$ ($205\text{ mm}$) | Distancia entre eje de rotación de hombro ($J_1$) y codo ($J_2$) |
| **Longitud Eslabón 2** | $L_2$ | $0.130\text{ m}$ ($130\text{ mm}$) | Distancia entre eje de rotación de codo ($J_2$) y eje del prismático ($J_3$) |
| **Alcance Máximo Horizontal** | $R_{\max} = L_1 + L_2$ | $0.335\text{ m}$ ($335\text{ mm}$) | Radio exterior de la frontera del espacio de trabajo |
| **Alcance Mínimo Horizontal** | $R_{\min} = \|L_1 - L_2\|$ | $0.075\text{ m}$ ($75\text{ mm}$) | Radio interior de la zona muerta singular |
| **Cota Superior Z (Reposo)** | $Z_{\text{HOME}}$ | $0.080\text{ m}$ ($80\text{ mm}$) | Posición retraída del carro prismático |
| **Cota Inferior Z (Extensión)**| $Z_{\max}$ | $0.170\text{ m}$ ($170\text{ mm}$) | Extensión máxima operativa con pinza |
| **Carrera Prismática Efectiva**| $\Delta Z$ | $0.090\text{ m}$ ($90\text{ mm}$) | Carrera de descenso vertical útil |
| **Carrera Mecánica Total Cremallera**| $\Delta Z_{\text{mec}}$ | $0.150\text{ m}$ ($150\text{ mm}$) | Longitud física medida en malla CAD STL de cremallera |

---

## 2. Modelo Cinemático Directo (FK)

Se establece el marco de referencia inercial $\{0\}$ en la base del robot, donde el eje $+X$ apunta hacia el frente del robot y $+Y$ hacia la izquierda, manteniendo $+Z$ alineado verticalmente hacia arriba según la regla de la mano derecha.

### Transformación y Ecuaciones Cartesianas
Dado el vector articular $\mathbf{q} = [q_1, q_2, q_3]^T$:
- El ángulo absoluto del primer eslabón con respecto al eje inercial $X$ se define con un desfase de origen en $+Y$:
  $$\theta_1 = q_1 + \frac{\pi}{2}$$
- El ángulo absoluto del segundo eslabón respecto al eje inercial $X$:
  $$\theta_2 = \theta_1 + q_2 = q_1 + q_2 + \frac{\pi}{2}$$

Las coordenadas cartesianas del efector final $(x, y, z)$ se expresan mediante:
$$x = L_1 \cos(\theta_1) + L_2 \cos(\theta_2)$$
$$y = L_1 \sin(\theta_1) + L_2 \sin(\theta_2)$$
$$z = \text{clamp}(q_3, Z_{\text{HOME}}, Z_{\max})$$

Sustituyendo $\theta_1$ y $\theta_2$:
$$x = -L_1 \sin(q_1) - L_2 \sin(q_1 + q_2)$$
$$y = L_1 \cos(q_1) + L_2 \cos(q_1 + q_2)$$
$$z = q_3$$

---

## 3. Modelo Cinemático Inverso (IK)

El problema cinemático inverso consiste en determinar los ángulos articulares $(q_1, q_2)$ y la elongación $q_3$ necesarios para ubicar el efector final en una coordenada cartesiana objetivo $(x, y, z)$.

### 3.1. Distancia Radial y Validación de Espacio de Trabajo
La distancia radial horizontal al objetivo es:
$$d^2 = x^2 + y^2$$

Condición de existencia de solución cinemática:
$$(L_1 - L_2)^2 \le d^2 \le (L_1 + L_2)^2$$
Si $d^2 < (0.075)^2$ o $d^2 > (0.335)^2$, el punto objetivo está fuera del alcance físico del manipulador y el algoritmo rechaza la consigna para evitar singularidades incontrolables.

### 3.2. Deducción del Ángulo de Codo $q_2$
Aplicando el teorema del coseno en el triángulo formado por los eslabones $L_1$, $L_2$ y la distancia $d$:
$$d^2 = L_1^2 + L_2^2 - 2 L_1 L_2 \cos(\pi - q_2) = L_1^2 + L_2^2 + 2 L_1 L_2 \cos(q_2)$$

Despejando $\cos(q_2)$:
$$\cos(q_2) = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}$$

Para evitar indeterminaciones numéricas por precisión de coma flotante:
$$\cos(q_2) \leftarrow \max(-1.0, \min(1.0, \cos(q_2)))$$

El término $\sin(q_2)$ introduce la bifurcación de configuraciones (codo izquierdo vs. codo derecho):
$$\sin(q_2) = \sigma \sqrt{1 - \cos^2(q_2)}$$
Donde:
- $\sigma = -1.0$: **Codo Izquierdo** (Configuración física real predeterminada del robot en la celda).
- $\sigma = +1.0$: **Codo Derecho**.

Finalmente:
$$q_2 = \text{atan2}(\sin(q_2), \cos(q_2))$$

### 3.3. Deducción del Ángulo de Hombro $q_1$
Definiendo el ángulo polar del punto en el plano cartesiano:
$$\psi = \text{atan2}(y, x)$$

El ángulo interno $\beta$ entre el eslabón $L_1$ y la línea radial $d$:
$$\beta = \text{atan2}(L_2 \sin(q_2), L_1 + L_2 \cos(q_2))$$

Por lo tanto:
$$\theta_1 = \psi - \beta$$
$$q_1 = \text{normalize\_angle}\left(\theta_1 - \frac{\pi}{2}\right)$$

### 3.4. Eje Prismático $q_3$
$$q_3 = \max(Z_{\text{HOME}}, \min(Z_{\max}, z))$$

---

## 4. Análisis de Singularidades y Espacio de Trabajo

El jacobiano geométrico plano del SCARA en el plano $XY$ está dado por:
$$J_{xy} = \begin{bmatrix}
-\left(L_1 \sin\theta_1 + L_2 \sin\theta_2\right) & -L_2 \sin\theta_2 \\
L_1 \cos\theta_1 + L_2 \cos\theta_2 & L_2 \cos\theta_2
\end{bmatrix}$$

El determinante del Jacobiano es:
$$\det(J_{xy}) = L_1 L_2 \left(-\sin\theta_1 \cos\theta_2 + \cos\theta_1 \sin\theta_2\right) = L_1 L_2 \sin(\theta_2 - \theta_1) = L_1 L_2 \sin(q_2)$$

### Tipos de Singularidades:
1. **Singularidad de Extensión Máxima ($q_2 = 0$):**
   - Brazo totalmente estirado a $335\text{ mm}$. $\det(J) = 0$. Se pierde la capacidad de acelerar radialmente hacia afuera.
2. **Singularidad de Retracción Interna ($q_2 = \pm \pi$):**
   - Brazo totalmente plegado sobre sí mismo a $75\text{ mm}$. $\det(J) = 0$.
3. **Manejo en Software:**
   - La planificación de trayectoria incluye un umbral de seguridad $\epsilon = 1\times 10^{-5}\text{ m}^2$ en las cotas de alcance para evitar división por cero o matrices mal condicionadas.

---

## 5. Planificación de Trayectorias y Control Articular

### 5.1. Perfil Suave S-Curve (Minimum Jerk)
Para garantizar aceleraciones continuas y eliminar sobreimpulsos mecánicos que desalineen la transmisión por correas dentadas GT2:
$$\tau = \frac{t - t_0}{T}, \quad \tau \in [0, 1]$$
$$s(\tau) = \frac{1}{2}\left(1 - \cos(\pi \tau)\right)$$

Posición cartesiana instantánea deseada:
$$\mathbf{p}_d(t) = \mathbf{p}_0 + \left(\mathbf{p}_f - \mathbf{p}_0\right) s(\tau)$$

### 5.2. Cálculo Automático de Tiempo de Recorrido ($T$)
El tiempo de trayectoria se calcula dinámicamente según la distancia euclidiana y las velocidades máximas articulares físicas:
$$d_{\text{cart}} = \sqrt{(x_f - x_0)^2 + (y_f - y_0)^2 + (z_f - z_0)^2}$$
$$t_{\text{cart}} = \frac{d_{\text{cart}}}{v_{\text{des}}}, \quad v_{\text{des}} = 0.080\text{ m/s}$$
$$t_{j1} = \frac{|\Delta q_1|}{\omega_{\max, 1}}, \quad t_{j2} = \frac{|\Delta q_2|}{\omega_{\max, 2}}, \quad t_{j3} = \frac{|\Delta z|}{v_{\max, 3}}$$
$$T = \max\left(t_{\text{cart}}, t_{j1}, t_{j2}, t_{j3}, 0.8\text{ s}\right)$$

### 5.3. Control Híbrido Feedforward + Feedback PI
La consigna de velocidad enviada a cada motor en tiempo real combina:
$$\omega_{\text{motor}, i} = s_i \cdot \left(\omega_{ff, i} + K_{p, i} e_i + K_{i, i} \int e_i dt\right) \cdot N_i$$
Donde $s_i \in \{-1, +1\}$ es el factor de convención de signo físico y $N_i$ es la relación de reducción mecánica.

---

## 6. Modelado de Transmisiones y Calibración de Encoders

Los tres ejes principales son accionados por motores DC con reductor planetario **JGA25-370 (12V, 60 RPM)** equipados con encoders de efecto Hall en cuadratura magnética:
- Pulsos por revolución del eje de salida del motor: $P_{\text{motor}} = 990\text{ pulsos/rev}$.

### 6.1. Articulación 1 — Hombro ($J_1$)
- Transmisión por poleas dentadas GT2:
  - Polea motriz: $Z_m = 20\text{ dientes}$.
  - Polea conducida: $Z_c = 80\text{ dientes}$.
  - Relación de reducción: $N_1 = \frac{80}{20} = 4.0$.
- Pulsos por revolución de la articulación:
  $$P_{J1} = 990 \times 4.0 = 3960\text{ pulsos/rev}$$
- Factor de conversión a radianes:
  $$K_{\text{enc}, 1} = \frac{3960}{2\pi} \approx 630.2535\text{ ticks/rad}$$

### 6.2. Articulación 2 — Codo ($J_2$)
- Transmisión por poleas dentadas GT2:
  - Polea motriz: $Z_m = 20\text{ dientes}$.
  - Polea conducida: $Z_c = 40\text{ dientes}$.
  - Relación de reducción: $N_2 = \frac{40}{20} = 2.0$.
- Pulsos por revolución de la articulación:
  $$P_{J2} = 990 \times 2.0 = 1980\text{ pulsos/rev}$$
- Factor de conversión a radianes:
  $$K_{\text{enc}, 2} = \frac{1980}{2\pi} \approx 315.1268\text{ ticks/rad}$$

### 6.3. Articulación 3 — Eje Prismático Z ($J_3$)
- Mecanismo: Piñón y cremallera de módulo 2:
  - Número de dientes del piñón: $Z_p = 20\text{ dientes}$.
  - Diámetro primitivo: $D_p = m \cdot Z_p = 2\text{ mm} \times 20 = 40\text{ mm} = 0.040\text{ m}$.
  - Avance lineal por revolución del piñón:
    $$\Delta L_{\text{rev}} = \pi \cdot D_p = \pi \times 40\text{ mm} \approx 125.6637\text{ mm/rev} = 0.1256637\text{ m/rev}$$
- Factor de conversión lineal (ticks por metro):
  $$K_{\text{enc}, 3} = \frac{P_{\text{motor}}}{\Delta L_{\text{rev}}} = \frac{990\text{ ticks}}{0.1256637\text{ m}} \approx 7878.07\text{ ticks/m}$$
  - Valor calibrado en el nodo de control: **$7878.0\text{ ticks/m}$**.

---

## 7. Dimensionamiento Eléctrico y Térmico del Gripper N20 y L298N

El efector final emplea un micromotor **N20** con reductora metálica y husillo roscado para el movimiento lineal de las mordazas:
- Tensión nominal de diseño del motor N20: $V_{\text{nom}} = 3.0\text{V DC}$.
- Tensión de alimentación del bus de potencia: $V_{\text{fuente}} = 12.0\text{V DC}$.
- Driver: Puente H bipolar integrado **L298N**.

### 7.1. Caída de Tensión en el Puente Bipolar L298N
A diferencia de los puentes MOSFET modernos, los transistores bipolares NPN y PNP en puente Darlington del L298N introducen una caída de saturación acumulada ($V_{CE,\text{sat}}$ superior e inferior):
$$V_{\text{drop}} \approx 1.8\text{V} - 2.0\text{V}$$
Por lo tanto, la tensión pico efectiva en bornes de la carga inductiva durante el estado alto del pulso es:
$$V_{\text{pico}} = V_{\text{fuente}} - V_{\text{drop}} = 12.0\text{V} - 2.0\text{V} = 10.0\text{V}$$

### 7.2. Cálculo del Ciclo Útil (PWM Duty Cycle) Seguro
La tensión media equivalente aplicada al motor responde a la relación del ciclo de trabajo $D = \frac{\text{PWM}\%}{100}$:
$$V_{\text{rms}} \approx V_{\text{med}} = D \cdot V_{\text{pico}}$$

Despejando el ciclo de trabajo máximo para no superar la tensión de aislamiento del motor ($3.0\text{V}$):
$$D_{\max} = \frac{3.0\text{V}}{10.0\text{V}} = 0.30 \implies \mathbf{30.0\%}$$

### 7.3. Valores Operativos Calibrados
1. **Apertura de Mordazas (`OPEN`):**
   - Ciclo de trabajo: $D = 25.0\% \implies V_{\text{med}} = 2.50\text{V}$.
   - Tiempo de pulso: $T = 400\text{ ms}$.
   - Resultado: Apertura total sin fatiga de topes mecánicos.
2. **Cierre de Agarre (`CLOSE`):**
   - Ciclo de trabajo: $D = 18.0\% \implies V_{\text{med}} = 1.80\text{V}$.
   - Tiempo de pulso: $T = 350\text{ ms}$.
   - Par generado: Suficiente para sostener de forma segura los bombones/chocolates ($m \approx 25\text{ g}$) sin aplastamiento y protegiendo el husillo de plástico 3D.
3. **Mecanismo de Seguridad por Software:**
   Tanto en el firmware de la ESP32 como en la GUI de ROS 2, se impone una función de acotamiento estricta:
   ```python
   pwm_efectivo = max(0.0, min(30.0, pwm_ingresado))
   ```
   Impidiendo cualquier fallo humano que queme el bobinado por sobretensión.
