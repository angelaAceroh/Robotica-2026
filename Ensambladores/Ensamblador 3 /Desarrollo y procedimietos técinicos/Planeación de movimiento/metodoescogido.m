% =========================================================
% ROBOT SCARA 3 GDL (CONFIGURACIÓN RRP)
% TRAYECTORIA: HOME -> PUNTO A -> BAJADA -> SUBIDA -> PUNTO B
% Estado articular q = [theta1 (rad), theta2 (rad), d3 (mm)]
% Estado cartesiano P = [x (mm), y (mm), z (mm)]
% Unidades: longitud = mm | tiempo = s | ángulos = rad
% =========================================================

clear; clc; close all;

% ---------------------------------------------------------
% 1. PARÁMETROS GEOMÉTRICOS DEL ROBOT SCARA (RRP)
% ---------------------------------------------------------
a1 = 350.0;   % Longitud del eslabón 1 (R1) [mm]
a2 = 250.0;   % Longitud del eslabón 2 (R2) [mm]
d1 = 500.0;   % Altura fija de la base al origen del eje Z [mm]
d_tool = 0.0; % Longitud de la herramienta final fijada al eje Z [mm]

% ---------------------------------------------------------
% 2. DEFINICIÓN DE PUNTOS DE LA TRAYECTORIA [X, Y, Z] mm
% ---------------------------------------------------------

% --- FASE 0: Posición de reposo ---
P_home     = [ 350.0;    0.0;  250.0]; % P1: Home

% --- FASE 1: Trayectoria desde Home hacia Punto A (Con puntos intermedios XY) ---
P_A_int1   = [ 340.0;  100.0;  250.0]; % P2: Intermedio 1 hacia A
P_A_int2   = [ 320.0;  180.0;  250.0]; % P3: Intermedio 2 hacia A
P_A_alto   = [ 300.0;  220.0;  250.0]; % P4: Punto A (Posición alta)

% --- FASE 2: Bajada en el Punto A (Con puntos intermedios en Z) ---
P_A_baj1   = [ 300.0;  220.0;  200.0]; % P5: Bajando (Paso 1)
P_A_baj2   = [ 300.0;  220.0;  150.0]; % P6: Bajando (Paso 2)
P_A_bajo   = [ 300.0;  220.0;  100.0]; % P7: Punto A Abajo (Contacto/Trabajo)

% --- FASE 3: Subida en el Punto A (Con puntos intermedios en Z) ---
P_A_sub1   = [ 300.0;  220.0;  150.0]; % P8: Subiendo (Paso 1)
P_A_sub2   = [ 300.0;  220.0;  200.0]; % P9: Subiendo (Paso 2)
P_A_ret    = [ 300.0;  220.0;  250.0]; % P10: Punto A Retorno (Posición alta)

% --- FASE 4: Traslado hacia el Punto B "otro lado" (Con puntos intermedios XY) ---
P_B_int1   = [ 150.0;  320.0;  250.0]; % P11: Intermedio 1 hacia B
P_B_int2   = [ -50.0;  330.0;  250.0]; % P12: Intermedio 2 hacia B
P_B_int3   = [-180.0;  280.0;  250.0]; % P13: Intermedio 3 hacia B
P_B_final  = [-250.0;  200.0;  250.0]; % P14: Punto B Destino Final

% Matriz con la secuencia completa de puntos de la ruta
PuntosTrabajo = [ ...
    P_home, ...
    P_A_int1, P_A_int2, P_A_alto, ...
    P_A_baj1, P_A_baj2, P_A_bajo, ...
    P_A_sub1, P_A_sub2, P_A_ret, ...
    P_B_int1, P_B_int2, P_B_int3, P_B_final ...
];

Ruta = PuntosTrabajo;

% ---------------------------------------------------------
% 3. VELOCIDADES Y PARÁMETROS DE CONTROL
% ---------------------------------------------------------
v_perforacion    = 31.42;   % mm/s (Movimiento descendente en Z)
v_salida         = 31.42;   % mm/s (Movimiento ascendente en Z)
v_desplazamiento = 104.72;  % mm/s (Movimiento en el plano XY)


dt     = 0.005;            % Paso de tiempo de integración [s]
Kp_pos = 4.0;              % Ganancia proporcional de posición

% Límites de velocidad de los motores
qdot_max = [ ...
    deg2rad(90.0);  % theta1_dot max [rad/s]
    deg2rad(90.0);  % theta2_dot max [rad/s]
    150.0           % d3_dot max [mm/s]
];

% Parámetros de autocorrección final por segmento
tol_pos       = 0.01;      % Tolerancia de posición [mm]
Kp_corr       = 10.0;      % Ganancia de corrección
max_iter_corr = 400;       % Iteraciones máximas de autocorrección


% =========================================================
% EJECUCIÓN PRINCIPAL Y CONTROL POR JACOBIANO
% =========================================================

% Cinemática Inversa Inicial
q = ik_scara_rrp(P_home, a1, a2, d1, d_tool, 1);

fprintf('==================================================\n');
fprintf(' CONFIGURACIÓN ARTICULAR INICIAL RRP (P_home)\n');
fprintf('==================================================\n');
fprintf('Theta 1 = %10.4f deg\n', rad2deg(q(1)));
fprintf('Theta 2 = %10.4f deg\n', rad2deg(q(2)));
fprintf('d3      = %10.4f mm\n', q(3));

% Comprobación Inicial
Pinicial = fk_scara_rrp(q, a1, a2, d1, d_tool);
fprintf('\nP inicial obtenido: [%.2f, %.2f, %.2f]\n', Pinicial(1), Pinicial(2), Pinicial(3));
fprintf('Error inicial posición = %.12f mm\n', norm(P_home - Pinicial));

% Historiales
t = 0.0;
t_hist         = t;
q_hist         = q';              % Matriz N x 3
qdot_hist      = [0, 0, 0];       % Matriz N x 3
p_hist         = Pinicial;        % Matriz 3 x N
pd_hist        = P_home;          % Matriz 3 x N
error_pos_hist = norm(P_home - Pinicial);

numero_segmentos = size(Ruta, 2) - 1;

fprintf('\n==================================================\n');
fprintf(' INICIANDO SEGUIMIENTO DE TRAYECTORIA Y AUTOCORRECCIÓN\n');
fprintf('==================================================\n');

for segmento = 1:numero_segmentos
    Pi = Ruta(:, segmento);
    Pf = Ruta(:, segmento + 1);
    deltaP = Pf - Pi;
    distancia = norm(deltaP);

    if distancia < 1e-12
        continue;
    end

    % Selección de velocidad por dirección en Z
    if deltaP(3) < -1e-9
        velocidad = v_perforacion;
        tipo = 'DESCENTE (BAJADA)';
    elseif deltaP(3) > 1e-9
        velocidad = v_salida;
        tipo = 'ASCENSO (SUBIDA)';
    else
        velocidad = v_desplazamiento;
        tipo = 'DESPLAZAMIENTO XY';
    end

    Tf = distancia / velocidad;
    N = max(1, ceil(Tf / dt));
    h = Tf / N;

    direccion = deltaP / distancia;
    v_ff = velocidad * direccion;

    fprintf('\nSegmento %2d : %-20s\n', segmento, tipo);
    fprintf('Pi = [%8.2f %8.2f %8.2f]\n', Pi(1), Pi(2), Pi(3));
    fprintf('Pf = [%8.2f %8.2f %8.2f]\n', Pf(1), Pf(2), Pf(3));
    fprintf('Distancia = %.3f mm | Tiempo estimado = %.4f s\n', distancia, Tf);

    % -----------------------------------------------------
    % Trayectoria Principal: Feed-Forward + Jacobiano Inverso
    % -----------------------------------------------------
    for k = 1:N
        s = k / N;
        Pd = Pi + s * (Pf - Pi);

        Pe = fk_scara_rrp(q, a1, a2, d1, d_tool);

        % Error en espacio cartesiano (3x1)
        ep = Pd - Pe;

        % Velocidad cartesiana comandada
        v_cmd = v_ff + Kp_pos * ep;

        % Jacobiano e Inversa
        J = jacobiano_scara_rrp(q, a1, a2);
        qdot = pinv(J) * v_cmd;

        % Saturación de velocidad articular
        saturacion = abs(qdot) ./ qdot_max;
        max_sat = max(saturacion);
        if max_sat > 1.0
            qdot = qdot / max_sat;
        end

        % Integración de Euler
        q = q + qdot * h;
        q(1) = atan2(sin(q(1)), cos(q(1)));
        q(2) = atan2(sin(q(2)), cos(q(2)));

        % Recalcular posición
        Pe_nuevo = fk_scara_rrp(q, a1, a2, d1, d_tool);
        err_pos = norm(Pd - Pe_nuevo);

        t = t + h;
        t_hist(end+1, 1)         = t;
        q_hist(end+1, :)         = q';
        qdot_hist(end+1, :)      = qdot';
        p_hist(:, end+1)         = Pe_nuevo;
        pd_hist(:, end+1)        = Pd;
        error_pos_hist(end+1, 1) = err_pos;
    end

    % -----------------------------------------------------
    % Bucle de Autocorrección al Final del Segmento
    % -----------------------------------------------------
    iter_corr = 0;
    while iter_corr < max_iter_corr
        iter_corr = iter_corr + 1;
        Pe = fk_scara_rrp(q, a1, a2, d1, d_tool);

        ep = Pf - Pe;
        err_pos = norm(ep);

        if err_pos < tol_pos
            break;
        end

        v_corr = Kp_corr * ep;
        J = jacobiano_scara_rrp(q, a1, a2);
        qdot = pinv(J) * v_corr;

        saturacion = abs(qdot) ./ qdot_max;
        max_sat = max(saturacion);
        if max_sat > 1.0
            qdot = qdot / max_sat;
        end

        q = q + qdot * dt;
        q(1) = atan2(sin(q(1)), cos(q(1)));
        q(2) = atan2(sin(q(2)), cos(q(2)));

        Pe_nuevo = fk_scara_rrp(q, a1, a2, d1, d_tool);

        t = t + dt;
        t_hist(end+1, 1)         = t;
        q_hist(end+1, :)         = q';
        qdot_hist(end+1, :)      = qdot';
        p_hist(:, end+1)         = Pe_nuevo;
        pd_hist(:, end+1)        = Pf;
        error_pos_hist(end+1, 1) = norm(Pf - Pe_nuevo);
    end

    Pseg = fk_scara_rrp(q, a1, a2, d1, d_tool);
    fprintf('Iteraciones de autocorrección = %d\n', iter_corr - 1);
    fprintf('Error final del segmento = %.8f mm\n', norm(Pf - Pseg));
end


% =========================================================
% REPORTES Y RESULTADOS FINALES
% =========================================================
Pfinal = fk_scara_rrp(q, a1, a2, d1, d_tool);

fprintf('\n==================================================\n');
fprintf(' ESTADÍSTICAS GLOBALES DE SIMULACIÓN (RRP)\n');
fprintf('==================================================\n');
fprintf('Punto final alcanzado: [%.2f, %.2f, %.2f]\n', Pfinal(1), Pfinal(2), Pfinal(3));
fprintf('Error final posición: %.8f mm\n', norm(Ruta(:, end) - Pfinal));
fprintf('Tiempo total ejecución: %.4f s\n', t_hist(end));
fprintf('Error máximo de posición: %.6f mm\n', max(error_pos_hist));
fprintf('Error promedio de posición: %.6f mm\n', mean(error_pos_hist));
fprintf('Error RMS de posición: %.6f mm\n', sqrt(mean(error_pos_hist.^2)));

% =========================================================
% EXPORTAR CSV
% =========================================================
q_unwrap = q_hist;
q_unwrap(:, 1) = unwrap(q_hist(:, 1));
q_unwrap(:, 2) = unwrap(q_hist(:, 2));

tabla_export = table(t_hist, q_unwrap(:, 1), q_unwrap(:, 2), q_unwrap(:, 3), ...
    'VariableNames', {'tiempo', 'q1_rad', 'q2_rad', 'd3_mm'});
writetable(tabla_export, 'trayectoria_scara_rrp.csv');
fprintf('\nArchivo exportado exitosamente: ''trayectoria_scara_rrp.csv''\n');

% =========================================================
% GRAFICACIÓN
% =========================================================

% 1. Trayectoria Cartesiana 3D
figure('Name', 'Trayectoria 3D SCARA RRP', 'Color', 'w');
plot3(p_hist(1, :), p_hist(2, :), p_hist(3, :), 'b-', 'LineWidth', 2, 'DisplayName', 'Trayectoria Real');
hold on;
plot3(Ruta(1, :), Ruta(2, :), Ruta(3, :), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Trayectoria Deseada');
scatter3(PuntosTrabajo(1, :), PuntosTrabajo(2, :), PuntosTrabajo(3, :), 50, 'k', 'filled', 'DisplayName', 'Puntos Clave');

% Etiquetas explicativas en las etapas
text(P_home(1)+10, P_home(2), P_home(3), '1. Home', 'FontSize', 8, 'FontWeight', 'bold');
text(P_A_alto(1)+10, P_A_alto(2), P_A_alto(3), '2. Llega a Punto A', 'FontSize', 8, 'FontWeight', 'bold');
text(P_A_bajo(1)+10, P_A_bajo(2), P_A_bajo(3), '3. Baja en A', 'FontSize', 8, 'FontWeight', 'bold');
text(P_B_final(1)+10, P_B_final(2), P_B_final(3), '4. Llega a Punto B (Fin)', 'FontSize', 8, 'FontWeight', 'bold');

xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('Trayectoria SCARA RRP: Home \rightarrow Punto A (Baja/Suba) \rightarrow Punto B');
grid on; axis equal; legend('Location', 'best');
view(45, 25);
hold off;

% 2. Posiciones Articulares vs Tiempo
figure('Name', 'Variables Articulares', 'Color', 'w');
subplot(3, 1, 1);
plot(t_hist, rad2deg(q_hist(:, 1)), 'b', 'LineWidth', 1.5);
ylabel('\theta_1 [grados]');
title('Variables Articulares vs Tiempo');
grid on;

subplot(3, 1, 2);
plot(t_hist, rad2deg(q_hist(:, 2)), 'g', 'LineWidth', 1.5);
ylabel('\theta_2 [grados]');
grid on;

subplot(3, 1, 3);
plot(t_hist, q_hist(:, 3), 'r', 'LineWidth', 1.5);
ylabel('d_3 [mm]');
xlabel('Tiempo [s]');
grid on;

% 3. Error de Seguimiento vs Tiempo
figure('Name', 'Error de Seguimiento', 'Color', 'w');
plot(t_hist, error_pos_hist, 'Color', [0.8 0.1 0.1], 'LineWidth', 1.4);
xlabel('Tiempo [s]');
ylabel('Error de Posición [mm]');
title('Error de Seguimiento Cartesiano');
grid on;


% =========================================================
% FUNCIONES LOCALES (CINEMÁTICA Y JACOBIANO)
% =========================================================

function P = fk_scara_rrp(q, a1, a2, d1, d_tool)
    % Cinemática Directa SCARA RRP: q = [theta1, theta2, d3]
    th1 = q(1);
    th2 = q(2);
    d3  = q(3);

    x = a1 * cos(th1) + a2 * cos(th1 + th2);
    y = a1 * sin(th1) + a2 * sin(th1 + th2);
    z = d1 - d3 - d_tool; % d3 positivo baja la herramienta

    P = [x; y; z];
end

function J = jacobiano_scara_rrp(q, a1, a2)
    % Jacobiano Geométrico 3x3 para SCARA RRP
    th1 = q(1);
    th2 = q(2);
    s1  = sin(th1);
    c1  = cos(th1);
    s12 = sin(th1 + th2);
    c12 = cos(th1 + th2);

    J = [ -a1*s1 - a2*s12,  -a2*s12,   0.0;
           a1*c1 + a2*c12,   a2*c12,   0.0;
               0.0,            0.0,   -1.0 ];
end

function q = ik_scara_rrp(P, a1, a2, d1, d_tool, c2_sign)
    % Cinemática Inversa Analítica SCARA RRP
    if nargin < 6
        c2_sign = 1; % 1: codo abajo/derecha, -1: codo arriba/izquierda
    end

    x = P(1);
    y = P(2);
    z = P(3);

    % Eslabón 3 Prismático
    d3 = d1 - z - d_tool;

    % Eslabones 1 y 2 Revolutas (Plano XY)
    r_sq = x^2 + y^2;
    c2 = (r_sq - a1^2 - a2^2) / (2.0 * a1 * a2);
    c2 = max(-1.0, min(1.0, c2));

    if abs(c2) == 1.0 && r_sq > (a1 + a2)^2
        error('Punto fuera del alcance del robot SCARA.');
    end

    s2 = c2_sign * sqrt(1.0 - c2^2);
    th2 = atan2(s2, c2);
    th1 = atan2(y, x) - atan2(a2 * s2, a1 + a2 * c2);

    % Normalizar ángulos en [-pi, pi]
    th1 = atan2(sin(th1), cos(th1));
    th2 = atan2(sin(th2), cos(th2));

    q = [th1; th2; d3];
end