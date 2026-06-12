close all
% -------------- 40 deg ---------------------------
clc
clear all

load('C:\Users\ester\Desktop\Sea-stand\RECUPERO_LAB1\results_LAB1.mat');

t_raw = Data_Lab1.SS_nominal.deg40.pos.time;
y_raw = Data_Lab1.SS_nominal.deg40.pos.signals.values;

t_shifted = t_raw + 1; 
t_plot = [0; t_shifted];
y_plot = [y_raw(1); y_raw]; 
r_plot = zeros(size(t_plot)); % Vettore di zeri lungo quanto t_plot
r_plot(t_plot >= 1) = 40;     % Imposta a 40 tutti i valori dopo 1 secondo

figure(1)
stairs(t_plot, r_plot, 'b--', 'LineWidth', 1); hold on
plot(t_plot, y_plot, 'r', 'LineWidth', 1); hold on
grid on
xlim([0 5])      % "Taglia" il grafico a 5 secondi
ylim([-5 55])    % Mantiene lo spazio per vedere bene la partenza a zero
title('Experimental Data: Nominal tracking (40 deg)')
xlabel('Time [s]')
ylabel('Position [deg]')
legend('Reference', 'Response', 'Location', 'southeast')

s = stepinfo(y_plot, t_plot ,'yfinal', 40, 'SettlingTimeThreshold',0.05);
disp( ' ---- 40 deg step info ----')
overshoot = s.Peak*100/40-100;
disp([s.SettlingTime, overshoot, s.Peak])

savefig('nominal40_lab')
% -------- 70 deg -----------------------------------------
clear all

load('C:\Users\ester\Desktop\Sea-stand\RECUPERO_LAB1\results_LAB1.mat');

t_raw = Data_Lab1.SS_nominal.deg70.pos.time;
y_raw = Data_Lab1.SS_nominal.deg70.pos.signals.values;

t_shifted = t_raw + 1; 
t_plot = [0; t_shifted];
y_plot = [y_raw(1); y_raw]; 
r_plot = zeros(size(t_plot)); % Vettore di zeri lungo quanto t_plot
r_plot(t_plot >= 1) = 70;     % Imposta a 40 tutti i valori dopo 1 secondo

figure(2)
stairs(t_plot, r_plot, 'b--', 'LineWidth', 1); hold on
plot(t_plot, y_plot, 'r', 'LineWidth', 1); hold on
grid on
xlim([0 5])      % "Taglia" il grafico a 5 secondi
ylim([-5 95])    % Mantiene lo spazio per vedere bene la partenza a zero
title('Experimental Data: Nominal tracking (70 deg)')
xlabel('Time [s]')
ylabel('Position [deg]')
legend('Reference', 'Response', 'Location', 'southeast')


s = stepinfo(y_plot, t_plot ,'yfinal', 70, 'SettlingTimeThreshold',0.05);
disp( ' ---- 70 deg step info ----')
overshoot = s.Peak*100/70-100;
disp([s.SettlingTime, overshoot, s.Peak])

savefig('nominal70_lab')
% ----------- 120 deg --------------------------------------------------

clear all

load('C:\Users\ester\Desktop\Sea-stand\RECUPERO_LAB1\results_LAB1.mat');

t_raw = Data_Lab1.SS_nominal.deg120.pos.time;
y_raw = Data_Lab1.SS_nominal.deg120.pos.signals.values;

t_shifted = t_raw + 1; 
t_plot = [0; t_shifted];
y_plot = [y_raw(1); y_raw]; 
r_plot = zeros(size(t_plot)); % Vettore di zeri lungo quanto t_plot
r_plot(t_plot >= 1) = 120;     % Imposta a 40 tutti i valori dopo 1 secondo

figure(3)
stairs(t_plot, r_plot, 'b--', 'LineWidth', 1); hold on
plot(t_plot, y_plot, 'r', 'LineWidth', 1); hold on
grid on
xlim([0 5])      % "Taglia" il grafico a 5 secondi
ylim([-5 165])    % Mantiene lo spazio per vedere bene la partenza a zero
title('Experimental Data: Nominal tracking (120 deg)')
xlabel('Time [s]')
ylabel('Position [deg]')
legend('Reference', 'Response', 'Location', 'southeast')


s = stepinfo(y_plot, t_plot ,'yfinal', 120, 'SettlingTimeThreshold',0.05);
disp( ' ---- 120 deg step info ----')
overshoot = s.Peak*100/120-100;
disp([s.SettlingTime, overshoot, s.Peak])

savefig('nominal120_lab')