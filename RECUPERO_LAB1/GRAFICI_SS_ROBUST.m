clear
clc
load("results_IA.mat"); % Simulazioni di Integral Action (6 da fare: con e senza friction 40,70,120 gradi) ho preso signals e error
load("results_LAB1.mat"); % Dati reali del LAB, sia per Integral Action che per Error Space (3 da fare uguale a sopra)


% 1. Estrai i dati dalla matrioska
t_IA_40_with  = Data_IA.deg40.with_tau.pos.time;
v_IA_40_with = Data_IA.deg40.with_tau.pos.signals.values;
t_IA_40_without  = Data_IA.deg40.without_tau.pos.time;
v_IA_40_without = Data_IA.deg40.without_tau.pos.signals.values;

figure;

% Sotto-grafico 1: Dati del Laboratorio
subplot(2,1,1); 
plot(t_IA_40_with, v_IA_40_with, 'b', 'LineWidth', 1.2);
grid on; title('With Friction Disturbance');
ylabel('Position [deg]');

% Sotto-grafico 2: Dati della Simulazione a casa
subplot(2,1,2); 
% (Assumendo il formato Timeseries dal blocco To Workspace)
plot(t_IA_40_without, v_IA_40_without, 'r', 'LineWidth', 1.2);
grid on; title('Without Friction Disturbance');
xlabel('Tempo [s]'); ylabel('Position [deg]');

% % 1. Estrai i dati dalla matrioska
% tempo  = Data_Lab1.SS_integral.deg40.pos.time;
% valori = Data_Lab1.SS_integral.deg40.pos.signals.values;
% 
% % 2. Crea i segnali di riferimento
% rif_costante = 40; 
% 
% % 3. Plotting
% figure;
% plot(tempo, valori, 'b-', 'LineWidth', 1.5); % Risposta del sistema (Blu)
% hold on;                                      % Attiva la sovrapposizione
% 
% yline(rif_costante, 'r--', 'LineWidth', 1.5);  % Riferimento Costante (Rosso Tratteggiato)
% %plot(tempo, rif_rampa, 'g-.', 'LineWidth', 1.5); % Riferimento Rampa (Verde Tratto-Punto)
% 
% % 4. Abbellimento del grafico
% grid on;
% xlabel('Tempo [s]');
% ylabel('Posizione');
% title('Confronto Risposta del Sistema e Riferimenti');
% 
% % La legenda segue l'ordine esatto in cui hai scritto i comandi di plot/yline
% legend('Risposta Sistema', 'Riferimento Costante', 'Location', 'best');