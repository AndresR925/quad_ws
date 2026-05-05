%% ==============================
%  ANÁLISIS DE ERROR CARTESIANO Y ESTABILIDAD
% ===============================
clear; clc; close all;

%% ==============================
% 1. CARGA DE DATOS
% ===============================
T1 = readtable("RESULTADOS2.xlsx", "Sheet", "Test1");
t = T1.t;
t = t - t(1); %Normalización del tiempo
t = t / 1000; % Conversión a segundos

roll  = deg2rad(str2double(T1.roll));
pitch = deg2rad(str2double(T1.pitch));
yaw   = deg2rad(str2double(T1.yaw));
dt = mean(diff(t));

%% ==============================
% 2. FILTRADO DE DATOS (BUTTERWOOTH)
% ===============================
alpha = 0.15; % Filtro paso bajo
roll_f  = filter(alpha, [1, -(1-alpha)], roll);
pitch_f = filter(alpha, [1, -(1-alpha)], pitch);
yaw_f   = filter(alpha, [1, -(1-alpha)], yaw);

%% ==============================
% 3. ELIMINACIÓN DE OFFSET DE LAS VARIABLES
% ===============================
roll_c  = roll_f  - mean(roll_f);
pitch_c = pitch_f - mean(pitch_f);
yaw_c   = yaw_f   - mean(yaw_f);

%% ==============================
% 4. VELOCIDAD PROMEDIO
% ===============================
d_total = 1.15; 
v_promedio = d_total / t(end);
fprintf('Velocidad promedio: %.4f m/s\n', v_promedio);

%% ==============================
% 5. ESTIMACIÓN Y CORRECCIÓN DEL DRIFT
% ===============================
p_drift = polyfit(t, rad2deg(yaw_f), 1);
drift_rate = p_drift(1); % Tasa de drift (deg/s)

yaw_corregido_deg = rad2deg(yaw_f) - (drift_rate * t);
yaw_corregido = deg2rad(yaw_corregido_deg);

%% ==============================
% 6. ERROR DE DESVIACIÓN LATERAL
% ===============================
% Desviación lateral en cada instante de tiempo
desviacion_lateral = d_total * sin(yaw_corregido - yaw_corregido(1));

% RMS del error lateral
rms_error_lat = rms(desviacion_lateral);

fprintf('Desviación lateral máxima: %.4f m\n', max(abs(desviacion_lateral)));
fprintf('RMS del error lateral: %.4f m\n', rms_error_lat);

%% ==============================
% 7. EULER VS TIEMPO (ESTABILIDAD)
% ===============================
figure('Name', 'Ángulos de Euler Filtrados y Centrados');
plot(t, rad2deg(roll_c), 'LineWidth', 1.5); hold on;
plot(t, rad2deg(pitch_c), 'LineWidth', 1.5);
plot(t, rad2deg(yaw_c), 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Ángulo (deg)');
legend('Roll', 'Pitch', 'Yaw');
title('Orientación vs Tiempo (Estabilidad)');

%% ==============================
% 8. VARIACIÓN Y DRIFT DE YAW
% ===============================
figure('Name', 'Drift del Yaw');
plot(t, rad2deg(yaw_f), 'b', 'LineWidth', 1.5); hold on;
plot(t, yaw_corregido_deg, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Yaw (deg)');
legend('Yaw Crudo', 'Yaw Corregido');
title('Estimación y corrección del Drift');

%% ==============================
% 9. ERROR LATERAL VS. TIEMPO
% ===============================
figure('Name', 'Error de desviación lateral');
plot(t, desviacion_lateral, 'Color', [0.85, 0.325, 0.098], 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)');
ylabel('Error Lateral (m)');
title('Error cartesiano respecto a la trayectoria recta');
%% ==============================
% 10. RECONSTRUCCIÓN ODOMÉTRICA
% ===============================
% a. Distancia incremental en cada paso de tiempo
ds = v_promedio * diff(t); 
ds = [0; ds]; % Ajustar tamaño

% b. Integración de la trayectoria basada en el ángulo Yaw real
x_path = cumsum(ds .* cos(yaw_corregido));
y_path = cumsum(ds .* sin(yaw_corregido));
% Trayectoria Ideal (Línea recta perfecta sobre el eje X)
x_ideal = [0, d_total];
y_ideal = [0, 0];

figure('Name', 'Trayectoria Reconstruida X-Y');
plot(x_ideal, y_ideal, 'k--', 'LineWidth', 2); hold on;
plot(x_path, y_path, 'b', 'LineWidth', 2);

% c. Marcar el punto final físico (Ground Truth)
plot(d_total, 0.126, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

grid on; axis equal;
xlabel('Avance Longitudinal (m)');
ylabel('Desviación Lateral (m)');
legend('Trayectoria Ideal', 'Trayectoria Estimada (IMU)', 'Medición física');
title('Comparativa de Trayectoria: Teórica vs. Sensor vs. Real');