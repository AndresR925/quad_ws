%% =========================
%ANÁLISIS DE DESEMPEÑO ENERGÉTICO
% =========================

clear; clc; close all;

%% -------------------------
% 1. PARÁMETROS
% -------------------------
V = 7.4; %Voltaje de la batería         
m = 3.354; %Masa del robot(Kg)         
g = 9.81;%Gravedad
d = 1.15;%Distancia recorrida en m 

capacidad_bateria_Ah = 4; %Capacidad de la batería

%% -------------------------
%2. CARGA DE DATOS EXCEL (CONSOLIDADO DE TODAS LAS PRUEBAS)
% -------------------------
T1 = readtable("RESULTADOS2.xlsx", "Sheet", "Test7");

t = T1.t;
I_servos = str2double(T1.i_servos);
I_Arduino = str2double(T1.i_arduino);
I_Arduino = abs(I_Arduino); %Corrección de mediciones negativas

% Normalización del tiempo y conversión a seg.
t = t - t(1);
t = t / 1000;

%% -------------------------
% 3. FILTRADO POR MEDIA MOVIL
% -------------------------
window = 10;
I_filt_servos = movmean(I_servos, window);
I_filt_arduino = movmean(I_Arduino, window);
%% CORRIENTE TOTAL
I_total = abs(I_filt_servos) + abs(I_filt_arduino);

%% -------------------------
% 4. POTENCIA
% -------------------------
P = V * I_total;
P_prom = mean(P); 
%% -------------------------
% 5. ENERGÍA POR MÉTODO DE INTERGACIÓN TRAPEZOIDAL
% -------------------------
energia = trapz(t, P);

%% -------------------------
% 6. Cost of Transport
% -------------------------
CoT = energia / (m * g * d);

%% -------------------------
% 7. Corriente promedio
% -------------------------
I_prom = mean(I_total);

%% -------------------------
% 8. Estimación de autonomía
% -------------------------
% Factor de descarga
factor_descarga = 0.8; 
capacidad_utilizable_Ah = capacidad_bateria_Ah * factor_descarga;

% Factor de servicio
I_eff = I_prom * 1.1; 
tiempo_autonomia_h = capacidad_utilizable_Ah / I_eff;
tiempo_autonomia_min = tiempo_autonomia_h * 60;

%% -------------------------
% 9. Comportamiento de la corriente
% -------------------------
I_max = max(I_total);
I_min = min(I_total);
I_std = std(I_total);
I_rms = rms(I_total);

%% -------------------------
% 10. Resultados.
% -------------------------
fprintf('\n===== RESULTADOS ENERGÉTICOS =====\n');
fprintf('🔋 Energía: %.2f J\n', energia);
fprintf('⚡ CoT: %.4f\n', CoT);
fprintf('⚡ Potencia Promedio: %.2f W\n', P_prom);
fprintf('🔌 I_prom: %.2f A\n', I_prom);
fprintf('📈 I_max: %.2f A\n', I_max);
fprintf('📉 I_min: %.2f A\n', I_min);
fprintf('📊 Std: %.2f A\n', I_std);
fprintf('📊 RMS: %.2f A\n', I_rms);
fprintf('⏱️ Autonomía: %.2f min\n', tiempo_autonomia_min);

%% -------------------------
% 11. Gráficas
% -------------------------

figure;
plot(t, I_filt_arduino, '--'); hold on;
plot(t, I_filt_servos, 'd'); hold on;
plot(t, I_total, 'LineWidth', 2);
legend('Corriente arduino', 'Corriente servomotores', 'Corriente total');
title('Corriente vs Tiempo');
xlabel('Tiempo [s]');
ylabel('Corriente [A]');
grid on;

figure;
plot(t, P);
title('Potencia vs Tiempo');
xlabel('Tiempo [s]');
ylabel('Potencia [W]');
grid on;

figure;
E_acum = cumtrapz(t, P);
plot(t, E_acum, 'LineWidth', 2);
title('Energía acumulada');
xlabel('Tiempo [s]');
ylabel('Energía [J]');
grid on;