%% PLOT
clear all; clc; close all

% Upload of the structure
load('results_LAB2_1_PID.mat','results');

% Choose data to compare
T = 0.001;      % Choose between: 0.001, 0.01, 0.05
arch = 'No_AW'; % Choose between: No_AW, With_AW

T_str = sprintf('T_%s', strrep(num2str(T), '.', '_'));
%% PLOT
clear all; clc; close all

% Upload of the structure
load('results_LAB2_1_PID.mat','results');

% Choose data to compare
T = 0.001;      % Choose between: 0.001, 0.01, 0.05
arch = 'No_AW'; % Choose between: No_AW, With_AW

T_str = sprintf('T_%s', strrep(num2str(T), '.', '_'));
title_str = ['Comparison of different methods with T = ', num2str(T), ' s and ', strrep(arch, '_', ' '), ' architecture'];

% position plot
time =  results.BE.(arch).(T_str).thl.time;
y1 = results.BE.(arch).(T_str).thl.signals.values;
y2 = results.FE.(arch).(T_str).thl.signals.values;
y3 = results.Tustin.(arch).(T_str).thl.signals.values;
y4 = results.Exact.(arch).(T_str).thl.signals.values;

% Plot
figure('Name',title_str)
hold on; grid on ;
stairs(time, y1, 'LineWidth',1.5)
stairs(time, y2, 'LineWidth',1.5)
stairs(time, y3, 'LineWidth',1.5)
stairs(time, y4, 'LineWidth',1.5)

title(['Risposta a gradino (\theta_l) - ', strrep(arch, '_', ' '), ' - T = ', num2str(T), 's']);
xlabel('Time [s]');
ylabel('Position [deg]');
legend('B. Euler', 'F. Euler', 'Tustin', 'Exact', 'Location', 'best');
% position plot
time =  results.BE.(arch).(T_str).thl.time;
y1 = results.BE.(arch).(T_str).thl.signals.values;
y2 = results.FE.(arch).(T_str).thl.signals.values;
y3 = results.Tustin.(arch).(T_str).thl.signals.values;
y4 = results.Exact.(arch).(T_str).thl.signals.values;

% Plot
figure('Name',title_str)
hold on; grid on ;
stairs(time, y1, 'LineWidth',1.5)
stairs(time, y2, 'LineWidth',1.5)
stairs(time, y3, 'LineWidth',1.5)
stairs(time, y4, 'LineWidth',1.5)

title(['Risposta a gradino (\theta_l) - ', strrep(arch, '_', ' '), ' - T = ', num2str(T), 's']);
xlabel('Time [s]');
ylabel('Position [deg]');
legend('B. Euler', 'F. Euler', 'Tustin', 'Exact', 'Location', 'best');