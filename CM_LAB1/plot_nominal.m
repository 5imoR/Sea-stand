close all
clc
clear all

% --- CARICAMENTO DATI ---
load('C:\Users\ester\Desktop\Sea-stand\RECUPERO_LAB1\results_LAB1.mat');

% --- ESTRAZIONE E PREPARAZIONE DATI ---
t_no = Data_Lab1.PID360.NO_AW.pos.time + 1;
y_no = Data_Lab1.PID360.NO_AW.pos.signals.values;

t_si = Data_Lab1.PID360.SI_AW.pos.time + 1;
y_si = Data_Lab1.PID360.SI_AW.pos.signals.values;

t_ref = [0, 1, 1, max(t_no)]; 
r_ref = [0, 0, 360, 360];

% --- PLOT ---
fig = figure(1);
clf; 
hold on; grid on;

plot(t_no, y_no, 'Color', [0.9290 0.6940 0.1250], 'LineStyle', '-', 'LineWidth', 0.5);
plot(t_si, y_si, 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'LineWidth', 0.5);
plot(t_ref, r_ref, 'Color', [0 0.4470 0.7410], 'LineStyle', ':', 'LineWidth', 0.5);

% --- LINEA DI SOGLIA ---
yline(360*1.1, 'Color', 'black', 'LineWidth', 0.4, 'LineStyle', '-.', ...
      'Label', 'Maximum overshoot required', 'LabelHorizontalAlignment', 'left', ...
      'LabelVerticalAlignment', 'top', 'FontSize', 5.5);

% --- CONFIGURAZIONE ASSI E FONT ---
ax = gca;
box off;                  % Toglie la chiusura in alto e a destra degli assi
ax.FontSize = 8;          

xlim([0 3])               
ax.YLim(1) = -5;          

xlabel('Time [s]', 'FontSize', 9)
ylabel('Position [deg]', 'FontSize', 9)

% --- LEGENDA ---
lgd = legend('Without Anti-windup', 'With Anti-windup', 'Reference',  ...
       'Location', 'southeast', 'FontSize', 8); 
lgd.Box = 'on';           % ---> RIMESSO: Forza la presenza del bordo attorno alla legenda <---

% --- IMPOSTAZIONI DIMENSIONI ED ESPORTAZIONE PDF ---
fig.Units = 'centimeters';
fig.Position = [5, 5, 12, 6.5]; 
fig.Color = 'w';

% Salva il file .fig
savefig('PID_REAL_360SIeNO_AW.fig')

% Esportazione perfetta per LaTeX
nome_file_pdf = 'PID_REAL_360SIeNO_AW.pdf';
exportgraphics(fig, nome_file_pdf, 'ContentType', 'vector', 'BackgroundColor', 'none');

% disp(['Grafico generato e salvato in PDF come: ', nome_file_pdf]);