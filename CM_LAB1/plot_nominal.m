close all
%------------------- 40 deg -------------------------- 
clc
clear all

% Load data
load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\40NoFR.mat');
load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\r40.mat');
fr = load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\40FR.mat');
tr = load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\40tr.mat');
load('C:\Users\ester\Desktop\Sea-stand\RECUPERO_LAB1\results_LAB1.mat');

% Variable assignment
y2 = fr.y;
t2 = tr.t; 
r2 = tr.r; 

t_raw = Data_Lab1.SS_nominal.deg40.pos.time;
y_raw = Data_Lab1.SS_nominal.deg40.pos.signals.values;

t_shifted = t_raw + 1; 
t_plot = [0; t_shifted];
y_plot = [y_raw(1); y_raw]; 
r_plot = zeros(size(t_plot)); % Vettore di zeri lungo quanto t_plot
r_plot(t_plot >= 1) = 40;     % Imposta a 40 tutti i valori dopo 1 secondo

figure(1);
hold on; grid on;

% 1. Reference: Blu classico di MATLAB (Linea a puntini)
% A schermo: Blu elegante. In stampa: Grigio Scuro.
plot(t, r, 'Color', [0 0.4470 0.7410], 'LineStyle', ':', 'LineWidth', 1.2);

% 2. Sim (No Friction): Giallo Oro / Senape (Linea tratteggiata)
% A schermo: Oro. In stampa: Grigio Chiaro. Perfetto per il dato "meno reale".
plot(t, y, 'Color', [0.9290 0.6940 0.1250], 'LineStyle', '--', 'LineWidth', 1.5);

% 3. Sim (With Friction): Rosso Mattone (Linea continua)
% A schermo: Rosso intenso. In stampa: Grigio Medio. Mettiamo la linea continua
% così le "increspature" dell'attrito si vedono perfettamente.
plot(t2, y2, 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'LineWidth', 1.5);

% 4. Experimental LAB: Nero puro (Linea continua, più spessa)
% Rimane Nero. Lo spessore a 2 lo fa dominare visivamente sul resto.
plot(t_plot, y_plot, 'Color', [0 0 0], 'LineStyle', '-', 'LineWidth', 1.5);

% Threshold lines (sottili e in secondo piano)
yline(40*1.05, ':k', 'Linewidth', 0.5)
yline(40*0.95, 'Color', 'black', 'LineWidth', 0.5, 'LineStyle', ':', 'Label', 'Settling time threshold','LabelHorizontalAlignment','left', 'FontSize', 6);
yline(40*1.1, 'Color', 'black', 'LineWidth', 0.5, 'LineStyle', '-.', 'Label', 'Maximum overshoot required','LabelHorizontalAlignment','left', 'FontSize', 6);

xlim([0 5]) 
ylim([-5 55]) 
xlabel('Time [s]')
ylabel('Position [deg]')

% Legenda
legend('Reference', 'Sim. (No Friction)', 'Sim. (With Friction)', 'Experimental (LAB)', 'Location', 'southeast')

savefig('nominal40')

%%
%------------------- 70 deg --------------------------
clc
clear all

% Load data
load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\70NoFR.mat');
load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\r70.mat');
fr = load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\70FR.mat');
tr = load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\70tr.mat');

% Variable assignment
y2 = fr.y;
t2 = tr.t; 
r2 = tr.r; 

figure(2);

% --- Plot 1: NO Static Friction ---
plot(t, r, 'b--', 'LineWidth', 1); hold on
plot(t, y, 'color', [1 0.8 0], 'LineWidth', 1); hold on 
plot(t2, y2, 'r', 'LineWidth', 1); hold on
yline(70*1.05, 'Color', 'black', 'LineWidth', 0.5, 'LineStyle', ':');
yline(70*0.95, ':k', 'Linewidth', 0.5, 'Label', 'Settling time threshold','LabelHorizontalAlignment','left', 'FontSize', 8)
yline(70*1.1, 'Color', 'black', 'LineWidth', 0.5, 'Label', 'Maximum overshoot required', 'LineStyle', '-.','LabelHorizontalAlignment','left', 'FontSize', 8);


grid on
xlim([0 5]) 
ylim([-5 95]) % Aggiunto per staccare il segnale dall'asse X
xlabel('Time [s]')
ylabel('Position [deg]')
legend('Reference', 'Response without static friction', 'Response with static friction', 'Location', 'southeast')

% General title
sgtitle('Nominal tracking of 70deg step reference')

savefig('nominal70_sim')

%------------------- 120 deg --------------------------
clc
clear all

% Load data
load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\120NoFR.mat');
load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\r120.mat');
fr = load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\120FR.mat');
tr = load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\120tr.mat');

% Variable assignment
y2 = fr.y;
t2 = tr.t; 
r2 = tr.r; 

figure(3);

plot(t, r, 'b--', 'LineWidth', 1); hold on
plot(t, y, 'color', [1 0.8 0], 'LineWidth', 1); hold on 
plot(t2, y2, 'r', 'LineWidth', 1); hold on
yline(120*0.95, 'Color', 'black', 'LineWidth', 0.5, 'Label', 'Settling time threshold', 'LineStyle', ':', 'LabelHorizontalAlignment','left', 'FontSize', 8);
yline(120*1.05, ':k', 'Linewidth', 0.5)
yline(120*1.1, 'Color', 'black', 'LineWidth', 0.5, 'Label', 'Maximum overshoot required', 'LineStyle', '-.','LabelHorizontalAlignment','left', 'FontSize', 8);

grid on
xlim([0 5]) 
ylim([-5 165]) % Aggiunto per staccare il segnale dall'asse X
xlabel('Time [s]')
ylabel('Position [deg]')
legend('Reference', 'Response without static friction', 'Response with static friction', 'Location', 'southeast')

% General title
sgtitle('Nominal tracking of 120deg step reference')

savefig('nominal120_sim')