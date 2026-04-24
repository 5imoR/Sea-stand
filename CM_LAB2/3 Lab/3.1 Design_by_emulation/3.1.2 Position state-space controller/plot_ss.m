%% PLOT
clear all; clc; close all

% Upload of the structure
load('results_LAB2_2_StateSpace.mat','results');

%% Choose data to compare T = 0.001
T1 = 0.001;      
track1 = 'Nominal_Tracking';
track2 = 'Robust_Tracking';

T_str = sprintf('T_%s', strrep(num2str(T1), '.', '_'));

% Concatenazione corretta in MATLAB usando le parentesi quadre
title_str = ['Comparison of different methods with T = ', num2str(T1), ' s'];

y1  = results.(track1).(T_str).thl_est;
y2 = results.(track2).(T_str).thl_est;


figure;
grid on;
hold on;
 
stairs(y1.time, y1.signals.values, 'g', 'LineWidth',1.2)
stairs(y2.time, y2.signals.values, 'b', 'LineWidth',1.2)

hold off;
lgd = legend( "Nominal Tracking", "Robust Tracking");
lgd.Location = "best";