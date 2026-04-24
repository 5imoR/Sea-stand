%% PLOT
clear all; clc; close all

% Upload of the structure
load('results_LAB2_1_PID.mat','results');

%% Choose data to compare T = 0.001
T1 = 0.001;      
arch = 'No_AW'; % Choose between: 'No_AW', 'With_AW'

T_str = sprintf('T_%s', strrep(num2str(T1), '.', '_'));

% Concatenazione corretta in MATLAB usando le parentesi quadre
title_str = ['Comparison of different methods with T = ', num2str(T1), ' s and ', strrep(arch, '_', ' '), ' architecture'];

% position plot
time = results.FE.(arch).(T_str).thl.time;
y1   = results.BE.(arch).(T_str).thl.signals.values(1:5001);
y2   = results.FE.(arch).(T_str).thl.signals.values;
y3   = results.Tustin.(arch).(T_str).thl.signals.values;
y4   = results.Exact.(arch).(T_str).thl.signals.values;

% Plot
figure('Name', title_str)
hold on; grid on;
stairs(time, y1, 'LineWidth', 1.5)
stairs(time, y2, 'LineWidth', 1.5)
stairs(time, y3, 'LineWidth', 1.5)
stairs(time, y4, 'LineWidth', 1.5)

title(['Risposta a gradino (\theta_l) - ', strrep(arch, '_', ' '), ' - T = ', num2str(T1), 's']);
xlabel('Time [s]');
ylabel('Position [deg]');
legend('B. Euler', 'F. Euler', 'Tustin', 'Exact', 'Location', 'best');

%% Choose data to compare T = 0.05
T2 = 0.01;      
arch = 'No_AW'; % Choose between: 'No_AW', 'With_AW'

T_str = sprintf('T_%s', strrep(num2str(T2), '.', '_'));

% Concatenazione corretta in MATLAB usando le parentesi quadre
title_str = ['Comparison of different methods with T = ', num2str(T2), ' s and ', strrep(arch, '_', ' '), ' architecture'];

% position plot
time = results.FE.(arch).(T_str).thl.time;
y1   = results.BE.(arch).(T_str).thl.signals.values;
y2   = results.FE.(arch).(T_str).thl.signals.values;
y3   = results.Tustin.(arch).(T_str).thl.signals.values;
y4   = results.Exact.(arch).(T_str).thl.signals.values;

% Plot
figure('Name', title_str)
hold on; grid on;
stairs(time, y1, 'LineWidth', 1.5)
stairs(time, y2, 'LineWidth', 1.5)
stairs(time, y3, 'LineWidth', 1.5)
stairs(time, y4, 'LineWidth', 1.5)

title(['Risposta a gradino (\theta_l) - ', strrep(arch, '_', ' '), ' - T = ', num2str(T1), 's']);
xlabel('Time [s]');
ylabel('Position [deg]');
legend('B. Euler', 'F. Euler', 'Tustin', 'Exact', 'Location', 'best');

%% Choose data to compare T = 0.05
T3 = 0.05;      
arch = 'No_AW'; % Choose between: 'No_AW', 'With_AW'

T_str = sprintf('T_%s', strrep(num2str(T3), '.', '_'));

% Concatenazione corretta in MATLAB usando le parentesi quadre
title_str = ['Comparison of different methods with T = ', num2str(T3), ' s and ', strrep(arch, '_', ' '), ' architecture'];

% position plot
time = results.FE.(arch).(T_str).thl.time(1:101);
y1   = results.BE.(arch).(T_str).thl.signals.values;
y2   = results.FE.(arch).(T_str).thl.signals.values(1:101);
y3   = results.Tustin.(arch).(T_str).thl.signals.values;
y4   = results.Exact.(arch).(T_str).thl.signals.values;

% Plot
figure('Name', title_str)
hold on; grid on;
stairs(time, y1, 'LineWidth', 1.5)
stairs(time, y2, 'LineWidth', 1.5)
stairs(time, y3, 'LineWidth', 1.5)
stairs(time, y4, 'LineWidth', 1.5)

title(['Risposta a gradino (\theta_l) - ', strrep(arch, '_', ' '), ' - T = ', num2str(T1), 's']);
xlabel('Time [s]');
ylabel('Position [deg]');
legend('B. Euler', 'F. Euler', 'Tustin', 'Exact', 'Location', 'best');

