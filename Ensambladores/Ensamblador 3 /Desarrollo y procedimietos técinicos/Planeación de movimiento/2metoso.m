 clear; close all;

% =========================================================
% ROBOT SCARA 3 GDL (RRP) - PERFILES DE VELOCIDAD
%   * Perfil CUADRADO     : velocidad constante en todo el segmento
%                           (salto instantáneo de 0 -> v y v -> 0)
%   * Perfil TRAPEZOIDAL  : aceleración - velocidad constante - desaceleración
%                           (si el tramo es corto se vuelve triangular)
% Control: Feed-Forward + Jacobiano inverso (igual que el código base)
% Unidades: mm | s | rad
% =========================================================

% ---------------------------------------------------------
% 1. PARÁMETROS GEOMÉTRICOS
% ---------------------------------------------------------
params.a1 = 350.0;
params.a2 = 250.0;
params.d1 = 500.0;
params.d_tool = 0.0;

% ---------------------------------------------------------
% 2. PUNTOS Y RUTA
% ---------------------------------------------------------
P1 = [300.0;  150.0;  200.0];
P2 = [300.0;  150.0;  100.0];
P3 = [400.0;  200.0;  200.0];
P4 = [400.0;  200.0;  100.0];
P5 = [450.0; -100.0;  200.0];
P6 = [450.0; -100.0;  100.0];

params.PuntosTrabajo = [P1, P2, P3, P4, P5, P6];
params.Ruta = [P1, P2, P1, P3, P4, P3, P5, P6, P5];

% ---------------------------------------------------------
% 3. VELOCIDADES MÁXIMAS (crucero) Y ACELERACIONES
% ---------------------------------------------------------
params.v_perforacion = 15.0;       % mm/s  (bajada en Z)
params.v_salida = 40.0;            % mm/s  (subida en Z)
params.v_desplazamiento = 120.0;   % mm/s  (plano XY)

% >>> Aceleraciones cartesianas para el perfil trapezoidal [mm/s^2] <<<
params.a_perforacion = 30.0;
params.a_salida = 80.0;
params.a_desplazamiento = 300.0;

params.dt = 0.005;
params.Kp_pos = 3.0;
params.qdot_max = [deg2rad(90.0); deg2rad(90.0); 150.0];
params.tol_pos = 0.01;
params.Kp_corr = 10.0;
params.max_iter_corr = 400;

% =========================================================
% EJECUCIÓN
% =========================================================
fprintf('%s\n', repmat('=', 1, 70));
fprintf(' PERFIL CUADRADO\n');
fprintf('%s\n', repmat('=', 1, 70));
Hc = simular('cuadrado', true, params);

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf(' PERFIL TRAPEZOIDAL\n');
fprintf('%s\n', repmat('=', 1, 70));
Ht = simular('trapezoidal', true, params);

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf(' COMPARACIÓN\n');
fprintf('%s\n', repmat('=', 1, 70));

sims = { 'Cuadrado', Hc; 'Trapezoidal', Ht };
for i = 1:size(sims, 1)
    nombre = sims{i,1};
    H = sims{i,2};
    qdot_deg12 = rad2deg(max(abs(H.qdot(1:2, :)), [], 'all'));
    qdot_mm3  = max(abs(H.qdot(3, :)));
    fprintf('%-12s | T total = %8.3f s | err max = %.4f mm | err RMS = %.4f mm | |qdot| max = %.1f deg/s, %.1f mm/s\n', ...
            nombre, H.t(end), max(H.err), sqrt(mean(H.err.^2)), qdot_deg12, qdot_mm3);
end

% =========================================================
% EXPORTAR CSV
% =========================================================
fprintf('\n');
exportar(Hc, 'trayectoria_scara_cuadrado.csv');
exportar(Ht, 'trayectoria_scara_trapezoidal.csv');
writetable(Hc.resumen, 'resumen_perfil_cuadrado.csv');
writetable(Ht.resumen, 'resumen_perfil_trapezoidal.csv');

% =========================================================
% GRÁFICAS
% =========================================================
% 1. Perfiles de velocidad / aceleración / posición de referencia
figure('Name', 'Perfiles de movimiento cartesiano', 'NumberTitle', 'off');
subplot(3, 1, 1);
plot(Hc.t, Hc.s_abs, 'r', 'LineWidth', 1.2); hold on;
plot(Ht.t, Ht.s_abs, 'b', 'LineWidth', 1.2);
ylabel('Distancia acumulada [mm]'); legend('Cuadrado', 'Trapezoidal'); grid on;

subplot(3, 1, 2);
plot(Hc.t, Hc.v_ref, 'r', 'LineWidth', 1.2); hold on;
plot(Ht.t, Ht.v_ref, 'b', 'LineWidth', 1.2);
ylabel('Velocidad de ref. [mm/s]'); legend('Cuadrado', 'Trapezoidal'); grid on;

subplot(3, 1, 3);
plot(Hc.t, Hc.a_ref, 'r', 'LineWidth', 1.2); hold on;
plot(Ht.t, Ht.a_ref, 'b', 'LineWidth', 1.2);
ylabel('Aceleración de ref. [mm/s²]'); xlabel('Tiempo [s]');
legend('Cuadrado', 'Trapezoidal'); grid on;
sgtitle('Perfiles de movimiento cartesiano');

% 2. Trayectoria 3D
figure('Name', 'Trayectoria cartesiana 3D', 'NumberTitle', 'off');
plot3(Hc.p(1,:), Hc.p(2,:), Hc.p(3,:), 'r', 'LineWidth', 1.5); hold on;
plot3(Ht.p(1,:), Ht.p(2,:), Ht.p(3,:), 'b', 'LineWidth', 1.5);
plot3(params.Ruta(1,:), params.Ruta(2,:), params.Ruta(3,:), 'k--', 'LineWidth', 1);
scatter3(params.PuntosTrabajo(1,:), params.PuntosTrabajo(2,:), params.PuntosTrabajo(3,:), 40, 'filled', 'k');
for i = 1:size(params.PuntosTrabajo, 2)
    text(params.PuntosTrabajo(1,i), params.PuntosTrabajo(2,i), params.PuntosTrabajo(3,i), sprintf(' P%d', i));
end
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('Trayectoria cartesiana 3D'); legend('Real - cuadrado', 'Real - trapezoidal', 'Deseada'); grid on; view(3);

% 3. Velocidades articulares
figure('Name', 'Velocidades articulares', 'NumberTitle', 'off');
etiquetas_v = {'\dot{\theta}_1 [deg/s]', '\dot{\theta}_2 [deg/s]', '\dot{d}_3 [mm/s]'};
for i = 1:3
    subplot(3, 1, i);
    fc = ifelse(i < 3, rad2deg(1), 1.0);
    plot(Hc.t, Hc.qdot(i,:) * fc, 'r', 'LineWidth', 1); hold on;
    plot(Ht.t, Ht.qdot(i,:) * fc, 'b', 'LineWidth', 1);
    ylabel(etiquetas_v{i}); grid on; legend('Cuadrado', 'Trapezoidal');
end
xlabel('Tiempo [s]');
sgtitle('Velocidades articulares');

% 4. Posiciones articulares
figure('Name', 'Posiciones articulares', 'NumberTitle', 'off');
etiquetas_p = {'\theta_1 [deg]', '\theta_2 [deg]', 'd_3 [mm]'};
for i = 1:3
    subplot(3, 1, i);
    fc = ifelse(i < 3, rad2deg(1), 1.0);
    plot(Hc.t, Hc.q(i,:) * fc, 'r', 'LineWidth', 1); hold on;
    plot(Ht.t, Ht.q(i,:) * fc, 'b', 'LineWidth', 1);
    ylabel(etiquetas_p{i}); grid on; legend('Cuadrado', 'Trapezoidal');
end
xlabel('Tiempo [s]');
sgtitle('Posiciones articulares');

% 5. Error de seguimiento
figure('Name', 'Error de seguimiento cartesiano', 'NumberTitle', 'off');
plot(Hc.t, Hc.err, 'r', 'LineWidth', 1.2); hold on;
plot(Ht.t, Ht.err, 'b', 'LineWidth', 1.2);
xlabel('Tiempo [s]'); ylabel('Error [mm]');
title('Error de seguimiento cartesiano'); legend('Cuadrado', 'Trapezoidal'); grid on;


% =========================================================
% FUNCIONES LOCALES DE CINEMÁTICA Y SIMULACIÓN
% =========================================================

function P = fk_scara_rrp(q, a1, a2, d1, d_tool)
    th1 = q(1); th2 = q(2); d3 = q(3);
    x = a1 * cos(th1) + a2 * cos(th1 + th2);
    y = a1 * sin(th1) + a2 * sin(th1 + th2);
    z = d1 - d3 - d_tool;
    P = [x; y; z];
end

function J = jacobiano_scara_rrp(q, a1, a2)
    th1 = q(1); th2 = q(2);
    s1 = sin(th1); c1 = cos(th1);
    s12 = sin(th1 + th2); c12 = cos(th1 + th2);
    J = [ -a1 * s1 - a2 * s12, -a2 * s12,  0.0;
           a1 * c1 + a2 * c12,  a2 * c12,  0.0;
                          0.0,        0.0, -1.0 ];
end

function q = ik_scara_rrp(P, a1, a2, d1, d_tool, c2_sign)
    if nargin < 6, c2_sign = 1; end
    x = P(1); y = P(2); z = P(3);
    d3 = d1 - z - d_tool;
    r_sq = x^2 + y^2;
    c2 = (r_sq - a1^2 - a2^2) / (2.0 * a1 * a2);
    if abs(c2) > 1.0
        error('Punto P fuera del alcance del robot.');
    end
    s2 = c2_sign * sqrt(1.0 - c2^2);
    th2 = atan2(s2, c2);
    th1 = atan2(y, x) - atan2(a2 * s2, a1 + a2 * c2);
    q = [atan2(sin(th1), cos(th1));
         atan2(sin(th2), cos(th2));
         d3];
end

function [T, f_func, info] = perfil_cuadrado(L, v)
    T = L / v;
    f_func = @(t) eval_cuadrado(t, v, T);
    info.T = T; info.ta = 0.0; info.tc = T; info.v_pico = v;
end

function [s, sdot, sddot] = eval_cuadrado(t, v, T)
    t = min(max(t, 0.0), T);
    s = v * t;
    sdot = v;
    sddot = 0.0;
end

function [T, f_func, info] = perfil_trapezoidal(L, v, a)
    if L >= (v^2 / a)
        ta = v / a;
        tc = (L - v^2 / a) / v;
        v_pico = v;
    else
        v_pico = sqrt(a * L);
        ta = v_pico / a;
        tc = 0.0;
    end
    T = 2.0 * ta + tc;
    f_func = @(t) eval_trapezoidal(t, L, v_pico, a, ta, tc, T);
    info.T = T; info.ta = ta; info.tc = tc; info.v_pico = v_pico;
end

function [s, sdot, sddot] = eval_trapezoidal(t, L, v_pico, a, ta, tc, T)
    t = min(max(t, 0.0), T);
    if t < ta
        s = 0.5 * a * t^2;
        sdot = a * t;
        sddot = a;
    elseif t < ta + tc
        s = 0.5 * a * ta^2 + v_pico * (t - ta);
        sdot = v_pico;
        sddot = 0.0;
    else
        td = T - t;
        s = L - 0.5 * a * td^2;
        sdot = a * td;
        sddot = -a;
    end
end

function H = simular(tipo_perfil, verbose, p)
    q = ik_scara_rrp(p.Ruta(:,1), p.a1, p.a2, p.d1, p.d_tool);
    P0 = fk_scara_rrp(q, p.a1, p.a2, p.d1, p.d_tool);
    t = 0.0;
    
    H.t = t;
    H.q = q;
    H.qdot = [0; 0; 0];
    H.p = P0;
    H.pd = p.Ruta(:,1);
    H.err = norm(p.Ruta(:,1) - P0);
    H.s_abs = 0.0;
    H.v_ref = 0.0;
    H.a_ref = 0.0;
    
    s_acum = 0.0;
    resumen = [];

    num_segs = size(p.Ruta, 2) - 1;
    for seg = 1:num_segs
        Pi = p.Ruta(:, seg);
        Pf = p.Ruta(:, seg + 1);
        deltaP = Pf - Pi;
        L = norm(deltaP);
        if L < 1e-12, continue; end
        u = deltaP / L;

        if deltaP(3) < -1e-9
            v = p.v_perforacion; a = p.a_perforacion; tipo = "PERFORACIÓN";
        elseif deltaP(3) > 1e-9
            v = p.v_salida; a = p.a_salida; tipo = "SALIDA";
        else
            v = p.v_desplazamiento; a = p.a_desplazamiento; tipo = "DESPLAZAMIENTO";
        end

        if strcmp(tipo_perfil, 'cuadrado')
            [T, f, info] = perfil_cuadrado(L, v);
        else
            [T, f, info] = perfil_trapezoidal(L, v, a);
        end

        N = max(1, ceil(T / p.dt));
        h = T / N;

        if verbose
            fprintf('Seg %2d %-15s L=%7.2f mm | T=%6.3f s | ta=%6.3f s | tc=%6.3f s | v_pico=%6.2f mm/s\n', ...
                    seg, tipo, L, T, info.ta, info.tc, info.v_pico);
        end

        item.segmento = seg; item.tipo = tipo; item.L = L;
        item.T = info.T; item.ta = info.ta; item.tc = info.tc; item.v_pico = info.v_pico;
        resumen = [resumen; item]; %#ok<AGROW>

        % ---- Seguimiento con perfil ----
        for k = 1:N
            tk = k * h;
            [s, sdot, sddot] = f(tk);
            Pd = Pi + s * u;
            v_ff = sdot * u;
            Pe = fk_scara_rrp(q, p.a1, p.a2, p.d1, p.d_tool);
            v_cmd = v_ff + p.Kp_pos * (Pd - Pe);
            J = jacobiano_scara_rrp(q, p.a1, p.a2);
            qdot = pinv(J) * v_cmd;
            m = max(abs(qdot) ./ p.qdot_max);
            if m > 1.0
                qdot = qdot / m;
            end
            q = q + qdot * h;
            q(1) = atan2(sin(q(1)), cos(q(1)));
            q(2) = atan2(sin(q(2)), cos(q(2)));
            Pn = fk_scara_rrp(q, p.a1, p.a2, p.d1, p.d_tool);
            t = t + h;

            H.t(end+1) = t;
            H.q(:, end+1) = q;
            H.qdot(:, end+1) = qdot;
            H.p(:, end+1) = Pn;
            H.pd(:, end+1) = Pd;
            H.err(end+1) = norm(Pd - Pn);
            H.s_abs(end+1) = s_acum + s;
            H.v_ref(end+1) = sdot;
            H.a_ref(end+1) = sddot;
        end

        s_acum = s_acum + L;

        % ---- Autocorrección final ----
        it = 0;
        while it < p.max_iter_corr
            Pe = fk_scara_rrp(q, p.a1, p.a2, p.d1, p.d_tool);
            ep = Pf - Pe;
            if norm(ep) < p.tol_pos, break; end
            it = it + 1;
            J = jacobiano_scara_rrp(q, p.a1, p.a2);
            qdot = pinv(J) * (p.Kp_corr * ep);
            m = max(abs(qdot) ./ p.qdot_max);
            if m > 1.0
                qdot = qdot / m;
            end
            q = q + qdot * p.dt;
            q(1) = atan2(sin(q(1)), cos(q(1)));
            q(2) = atan2(sin(q(2)), cos(q(2)));
            Pn = fk_scara_rrp(q, p.a1, p.a2, p.d1, p.d_tool);
            t = t + p.dt;

            H.t(end+1) = t;
            H.q(:, end+1) = q;
            H.qdot(:, end+1) = qdot;
            H.p(:, end+1) = Pn;
            H.pd(:, end+1) = Pf;
            H.err(end+1) = norm(Pf - Pn);
            H.s_abs(end+1) = s_acum;
            H.v_ref(end+1) = 0.0;
            H.a_ref(end+1) = 0.0;
        end
    end
    H.resumen = struct2table(resumen);
end

function exportar(H, nombre)
    qu = H.q';
    qu(:, 1) = unwrap(qu(:, 1));
    qu(:, 2) = unwrap(qu(:, 2));
    T_exp = table(H.t', qu(:,1), qu(:,2), qu(:,3), ...
        'VariableNames', {'tiempo', 'q1_rad', 'q2_rad', 'd3_mm'});
    writetable(T_exp, nombre);
    fprintf('Exportado: %s\n', nombre);
end

function res = ifelse(cond, valTrue, valFalse)
    if cond, res = valTrue; else, res = valFalse; end
end