%% ======================================================
%  LECTURA SERIAL DESDE ESP32 Y CREACIÓN DE ARCHIVO CSV
% ======================================================
clear; clc; close all;

%% 1️⃣ Configurar puerto serial
puerto = "COM5";
baudrate = 115200;

if ~isempty(serialportlist("available"))
    s = serialport(puerto, baudrate);
else
    error("No se detectó el puerto %s.", puerto);
end

configureTerminator(s,"LF");
disp("Esperando conexión...");
pause(2);

%% 2️⃣ Parámetros de adquisición
tiempo_adquisicion = 15;

distancia = [];
angulo = [];
Vref = [];
tiempo = [];

flush(s);

disp("Iniciando adquisición...");
tic;

%% 3️⃣ Monitoreo gráfico
figure('Name','Monitoreo ESP32','Color','w')

subplot(3,1,1)
h1 = animatedline('LineWidth',1.5);
title('Distancia (cm)')
grid on

subplot(3,1,2)
h2 = animatedline('LineWidth',1.5);
title('Ángulo (deg)')
grid on

subplot(3,1,3)
h3 = animatedline('LineWidth',1.5);
title('Voltaje referencia (V)')
grid on

%% 4️⃣ Lectura en tiempo real
while toc < tiempo_adquisicion

    if s.NumBytesAvailable > 0

        linea = readline(s);
        linea = strtrim(linea);

        datosStr = split(linea,",");

        if length(datosStr) == 3

            val_dist = str2double(datosStr(1));
            val_ang  = str2double(datosStr(2));
            val_ref  = str2double(datosStr(3));

            if ~isnan(val_dist) && ~isnan(val_ang) && ~isnan(val_ref)

                t_actual = toc;

                distancia(end+1) = val_dist;
                angulo(end+1) = val_ang;
                Vref(end+1) = val_ref;
                tiempo(end+1) = t_actual;

                addpoints(h1,t_actual,val_dist);
                addpoints(h2,t_actual,val_ang);
                addpoints(h3,t_actual,val_ref);

                drawnow limitrate
            end
        end
    end
end

disp("Adquisición finalizada.");

%% 5️⃣ Crear tabla
T = table(tiempo',distancia',angulo',Vref',...
    'VariableNames',{'Tiempo_s','Distancia_cm','Angulo_deg','Vref_V'});

%% 6️⃣ Guardar CSV
nombre_archivo = "datos_experimento_ref.csv";

writetable(T,nombre_archivo);

fprintf("Archivo '%s' creado con %d muestras\n",nombre_archivo,height(T));

%% 7️⃣ Cerrar puerto
clear s
disp("Puerto cerrado.");

%% Estadísticas
fprintf("\nResumen:\n");
fprintf("Ángulo promedio: %.2f°\n",mean(angulo));
fprintf("Distancia promedio: %.2f cm\n",mean(distancia));
fprintf("Voltaje referencia promedio: %.2f V\n",mean(Vref));