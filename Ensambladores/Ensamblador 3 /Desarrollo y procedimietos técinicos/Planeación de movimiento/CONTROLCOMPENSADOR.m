% =========================================================================
% Control Cinemático con Jacobiano Inverso y Límites de Velocidad - SCARA
% =========================================================================
clear; clc; close all;

%% 1. Parámetros Geométricos del Robot SCARA (m)
L1 = 0.20;    % Altura base (m) -> 20 cm
a1 = 0.165;   % Eslabón 1 (m)   -> 16.5 cm
a2 = 0.135;   % Eslabón 2 (m)   -> 13.5 cm

%% 2. Vector de Velocidades Máximas Articulares
% [theta1_dot_max (rad/s), theta2_dot_max (rad/s), d3_dot_max (m/s)]
q_dot_max = [1.5; 1.5; 0.15]; 

%% 3. Puntos Cartesianos del Efector Final (cm a m)
Puntos_cm = [
    5.5,  -6.5, 21.5;
  -10.0, -15.0, 10.0;
   19.0,   6.0, 15.0;
    0.0, -15.0, 10.0;
   23.5,   6.0, 15.0;
   10.0, -15.0, 10.0;
   28.5,   6.0, 15.0;
    5.5,  -6.5, 21.5
]';
Puntos_m = Puntos_cm / 100; % Conversión a metros
num_puntos = size(Puntos_m, 2);
num_segmentos = num_puntos - 1;

%% 4. Posición Articular Inicial (Cinemática Inversa SOLO para el primer punto)
x0 = Puntos_m(1, 1);
y0 = Puntos_m(2, 1);
z0 = Puntos_m(3, 1);

d3_0 = L1 - z0;
r2_0 = x0^2 + y0^2;
cos_th2_0 = max(min((r2_0 - a1^2 - a2^2) / (2 * a1 * a2), 1.0), -1.0);
th2_0 = acos(cos_th2_0);
th1_0 = atan2(y0, x0) - atan2(a2 * sin(th2_0), a1 + a2 * cos(th2_0));

q = [th1_0; th2_0; d3_0]; % Estado articular inicial

%% 5. Planificación de Tiempo y Trayectoria Cartesiana Lineal
v_cart_deseada = 0.12; % Velocidad lineal cartesiana objetivo (m/s)
T_segmentos = zeros(1, num_segmentos);
for i = 1:num_segmentos
    dist = norm(Puntos_m(:, i+1) - Puntos_m(:, i));
    T_segmentos(i) = max(dist / v_cart_deseada, 0.4);
end

T_total = sum(T_segmentos);
dt = 0.005; % Paso de tiempo (s)
t = 0:dt:T_total;
N = length(t);

%% 6. Matriz de Ganancia y Vectores de Almacenamiento
K = 15 * eye(3);

Q_hist     = zeros(3, N);
Q_dot_hist = zeros(3, N);
X_hist     = zeros(3, N);
X_d_hist   = zeros(3, N);
E_hist     = zeros(3, N);

t_acumulado = [0, cumsum(T_segmentos)];

%% 7. Bucle Principal de Simulación con Jacobiano Inverso
for k = 1:N
    tk = t(k);
    if tk >= T_total
        idx_seg = num_segmentos;
        t_local = T_segmentos(end);
    else
        idx_seg = find(tk >= t_acumulado(1:end-1), 1, 'last');
        t_local = tk - t_acumulado(idx_seg);
    end
    
    T_seg = T_segmentos(idx_seg);
    P_inicio = Puntos_m(:, idx_seg);
    P_fin    = Puntos_m(:, idx_seg + 1);
    
    % Interpolación lineal en Espacio Cartesiano (Trayectoria Recta 3D)
    s = t_local / T_seg;
    X_d = P_inicio + (P_fin - P_inicio) * s;
    X_dot_d = (P_fin - P_inicio) / T_seg;
    
    % Cinemática Directa Real
    th1 = q(1); th2 = q(2); d3 = q(3);
    xe = [
        a1 * cos(th1) + a2 * cos(th1 + th2);
        a1 * sin(th1) + a2 * sin(th1 + th2);
        L1 - d3
    ];
    
    % Error de Posición
    e = X_d - xe;
    
    % Jacobiano Analítico
    J_A = [
        -a1 * sin(th1) - a2 * sin(th1 + th2),  -a2 * sin(th1 + th2),   0;
         a1 * cos(th1) + a2 * cos(th1 + th2),   a2 * cos(th1 + th2),   0;
         0,                                     0,                    -1
    ];
    
    % Ley de Control usando Jacobiano Inverso
    q_dot_calculada = J_A \ (X_dot_d + K * e);
    
    % Saturación de Velocidad Articular
    q_dot = min(max(q_dot_calculada, -q_dot_max), q_dot_max);
    
    % Integración Numérica (Euler)
    q = q + q_dot * dt;
    
    % Guardar Histórico
    Q_hist(:, k)     = q;
    Q_dot_hist(:, k) = q_dot;
    X_hist(:, k)     = xe;
    X_d_hist(:, k)   = X_d;
    E_hist(:, k)     = e;
end

%% 8. Resultados en Consola
fprintf('========================================\n');
fprintf(' Simulación con Jacobiano Inverso Exitosa\n');
fprintf('========================================\n');
fprintf('Tiempo total estimado de recorrido: %.2f segundos\n', T_total);
fprintf('Posición Final Alcanzada:\n');
fprintf('X = %.2f cm | Y = %.2f cm | Z = %.2f cm\n', X_hist(1, end)*100, X_hist(2, end)*100, X_hist(3, end)*100);

%% 9. Generación de Gráficas Completas
% Figura 1: Análisis Dinámico y Cinemático (4 Subplots)
figure('Name', 'Control Cinemático SCARA con Jacobiano Inverso', 'Position', [80, 80, 1000, 900]);

% Subplot 1: Seguimiento Cartesiano
subplot(4, 1, 1);
plot(t, X_d_hist(1,:)*100, 'r--', t, X_hist(1,:)*100, 'r-', ...
     t, X_d_hist(2,:)*100, 'g--', t, X_hist(2,:)*100, 'g-', ...
     t, X_d_hist(3,:)*100, 'b--', t, X_hist(3,:)*100, 'b-', 'LineWidth', 1.5);
grid on;
legend('X_d', 'X_e', 'Y_d', 'Y_e', 'Z_d', 'Z_e', 'Location', 'eastoutside');
title('1. Seguimiento de Trayectoria Cartesiana (cm)');

% Subplot 2: Error de Posición
subplot(4, 1, 2);
plot(t, E_hist(1,:)*100, 'r', t, E_hist(2,:)*100, 'g', t, E_hist(3,:)*100, 'b', 'LineWidth', 1.5);
grid on;
legend('Error X', 'Error Y', 'Error Z', 'Location', 'eastoutside');
title('2. Evolución del Error de Posición (cm)');

% Subplot 3: Variables Articulares (Junturas)
subplot(4, 1, 3);
plot(t, Q_hist(1,:), 'r', t, Q_hist(2,:), 'g', t, Q_hist(3,:)*100, 'b', 'LineWidth', 1.5);
grid on;
legend('\theta_1 (rad)', '\theta_2 (rad)', 'd_3 (cm)', 'Location', 'eastoutside');
title('3. Variables Articulares / Junturas (Posición)');

% Subplot 4: Velocidades Articulares Reales vs Límites
subplot(4, 1, 4);
plot(t, Q_dot_hist(1,:), 'r', t, Q_dot_hist(2,:), 'g', t, Q_dot_hist(3,:), 'b', 'LineWidth', 1.5); hold on;
yline(q_dot_max(1), 'k--', 'Max \theta_1,\theta_2');
yline(-q_dot_max(1), 'k--');
yline(q_dot_max(3), 'm--', 'Max d_3');
yline(-q_dot_max(3), 'm--');
grid on;
legend('\dot{\theta}_1 (rad/s)', '\dot{\theta}_2 (rad/s)', '\dot{d}_3 (m/s)', 'Location', 'eastoutside');
title('4. Velocidades Articulares Evaluadas (Saturadas a V_{max})');
xlabel('Tiempo (s)');

% Figura 2: Visualización de Trayectoria 3D
figure('Name', 'Trayectoria 3D del Efector Final', 'Position', [150, 150, 750, 650]);
plot3(X_d_hist(1,:)*100, X_d_hist(2,:)*100, X_d_hist(3,:)*100, 'r--', 'LineWidth', 2); hold on;
plot3(X_hist(1,:)*100, X_hist(2,:)*100, X_hist(3,:)*100, 'b-', 'LineWidth', 1.2);
plot3(Puntos_cm(1,:), Puntos_cm(2,:), Puntos_cm(3,:), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y');

for i = 1:num_puntos
    text(Puntos_cm(1,i) + 0.8, Puntos_cm(2,i) + 0.8, Puntos_cm(3,i) + 0.8, ...
         ['P_{', num2str(i), '}'], 'FontSize', 9, 'FontWeight', 'bold');
end

grid on; box on;
xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
title('Trayectoria 3D Generada con Jacobiano Inverso');
legend('Trayectoria Deseada', 'Trayectoria Real', 'Waypoints', 'Location', 'northeastoutside');
axis equal;
view(45, 25);