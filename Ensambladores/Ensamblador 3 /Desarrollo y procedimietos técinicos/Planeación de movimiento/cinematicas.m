%% SCRIPT COMPLETO Y RÁPIDO: CINEMÁTICA DIRECTA E INVERSA ROBOT RPR
clear; clc; close all;

%% 1. DEMOSTRACIÓN SIMBÓLICA (Ecuaciones Explícitas)
disp('=====================================================');
disp(' 1. RESOLUCIÓN SIMBÓLICA DE LA CINEMÁTICA INVERSA');
disp('=====================================================');

syms th1 d2 th3 px py pz L1 L3 real

% Cinemática Directa (Posición)
px_eq = L3*cos(th1)*cos(th3) + d2*sin(th1);
py_eq = L3*sin(th1)*cos(th3) - d2*cos(th1);
pz_eq = L1 + L3*sin(th3);

% Despejes Simbólicos Analíticos
% 1. Articulación 3 (th3)
sin_th3 = (pz - L1) / L3;
cos_th3 = sqrt(1 - sin_th3^2);
sol_th3_sym = atan2(sin_th3, cos_th3);

% 2. Articulación 2 (d2)
k_sym = L3 * cos_th3;
sol_d2_sym = sqrt(px^2 + py^2 - k_sym^2);

% 3. Articulación 1 (th1)
sol_th1_sym = atan2(py, px) - atan2(-sol_d2_sym, k_sym);

disp('Expresiones simbólicas obtenidas:');
fprintf('  theta3 = '); disp(sol_th3_sym);
fprintf('  d2     = '); disp(sol_d2_sym);
fprintf('  theta1 = '); disp(sol_th1_sym);
disp(' ');


%% 2. PRUEBA NUMÉRICA Y VERIFICACIÓN
disp('=====================================================');
disp(' 2. PRUEBA NUMÉRICA CON VALORES REALES');
disp('=====================================================');

% Parámetros geométricos del robot
L1_val = 0.5; % metros
L3_val = 0.4; % metros

% Valores reales de prueba
th1_real = deg2rad(30); % 30°
d2_real  = 0.3;         % 0.3 m
th3_real = deg2rad(45); % 45°

% A) Cinemática Directa
px_val = double(subs(px_eq, {th1, d2, th3, L1, L3}, {th1_real, d2_real, th3_real, L1_val, L3_val}));
py_val = double(subs(py_eq, {th1, d2, th3, L1, L3}, {th1_real, d2_real, th3_real, L1_val, L3_val}));
pz_val = double(subs(pz_eq, {th1, d2, th3, L1, L3}, {th1_real, d2_real, th3_real, L1_val, L3_val}));

fprintf('Posición objetivo calculada:\n');
fprintf('  px = %.4f m | py = %.4f m | pz = %.4f m\n\n', px_val, py_val, pz_val);

% B) Cinemática Inversa
[soluciones, es_valido] = calcular_inversa_RPR(px_val, py_val, pz_val, L1_val, L3_val);

% C) Mostrar Resultados
if es_valido
    fprintf('Soluciones encontradas [theta1 (deg), d2 (m), theta3 (deg)]:\n');
    for k = 1:size(soluciones, 1)
        fprintf('  Solución %d: theta1 = %6.2f°,  d2 = %6.3f m,  theta3 = %6.2f°\n', ...
            k, rad2deg(soluciones(k,1)), soluciones(k,2), rad2deg(soluciones(k,3)));
    end
end


%% 3. GRAFICADO DE RESULTADOS
figure('Name', 'Cinemática Robot RPR', 'Color', 'w');
plot3(px_val, py_val, pz_val, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
grid on; hold on;
xlabel('Eje X (m)'); ylabel('Eje Y (m)'); zlabel('Eje Z (m)');
title('\bf\fontsize{14} Ecuaciones de Cinemática Directa', 'Interpreter', 'tex');
text(px_val, py_val, pz_val + 0.05, ' Efector Final', 'Interpreter', 'tex');


%% 4. FUNCIÓN LOCAL: CINEMÁTICA INVERSA NUMÉRICA
function [soluciones, valido] = calcular_inversa_RPR(px, py, pz, L1, L3)
    soluciones = [];
    valido = false;

    sin_theta3 = (pz - L1) / L3;

    if abs(sin_theta3) > 1
        warning('El punto solicitado está fuera del alcance geométrico en Z.');
        return;
    end

    val_theta3 = [atan2(sin_theta3, sqrt(1 - sin_theta3^2)), ...
                  atan2(sin_theta3, -sqrt(1 - sin_theta3^2))];

    for i = 1:length(val_theta3)
        th3 = val_theta3(i);
        k = L3 * cos(th3);
        arg_d2 = px^2 + py^2 - k^2;

        if arg_d2 >= 0
            valores_d2 = [sqrt(arg_d2), -sqrt(arg_d2)];

            for j = 1:length(valores_d2)
                d2 = valores_d2(j);
                th1 = atan2(py, px) - atan2(-d2, k);
                th1 = atan2(sin(th1), cos(th1)); % Normalizar a [-pi, pi]

                soluciones = [soluciones; th1, d2, th3];
            end
        end
    end

    if ~isempty(soluciones)
        valido = true;
    end
end