%% LABORATORIO 2 - DESIGN BY EMULATION
clear all, clc, close all

load ('../param.mat'); % motor parameters
load ('../est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)
Jeq = J_eq_est; Beq = B_eq_est; Tausf = tau_sf_est; 
load ('../PID_parameters.mat') % Kp, Ki, Kd and Tl (real derivative)
Tl = 3/2*Tl;

T = input('Select sample time (0.001, 0.01, 0.05) [s]: ');
arch_choice = input('Choose architecture  (1 = No anti-windup, 2 = With anti-windup): ');
method_choice = input('Choose discretization method (1 = B. Euler, 2 = F. Euler, 3 = Tustin): ');

if arch_choice == 1
    step_amplitude = 50; % step reference input [deg]
    Kw = 0; % anti-windup gain
elseif arch_choice == 2
    step_amplitude = 360; % step reference input [deg]
    Kw = 5/0.15;% anti-windup gain
end
%% Derivator implementation
% tustin 
sysC_deriv = tf([1, 0], [Tl, 1]); 
% s = tf('s');
% sysCd_deriv = s/(Tl*s)+1;
sysCd_deriv = c2d(sysC_deriv, T, 'tustin'); 
[N3, D3] = tfdata(sysCd_deriv, 'v');

Nd_options = [[1, -1]; [1, -1]; N3];
Dd_options = [[Tl+T, -Tl]; [Tl, T-Tl]; D3]; % 1:BE 2:FE 3:tustin
Nd = Nd_options(method_choice,:);
Dd = Dd_options(method_choice,:);


%% simulation 
out = sim("sim2_ester.slx");
disp(max(out.thl))

%% results: OVERSHOOT for T=0.01 and step_amplitude=360

% Kw            [BE]      |      [FE]     |    [Tustin]
% ----------------------------------------------------
% 150   --->  372.0814    |    397.3188   |    374.5253
% 90    --->  372.9670    |    397.9841   |    376.0435 
% 50    --->  377.0712    |    400.7490   |    382.1782
% 33.33 --->  386.1597    |   406.6603    |    394.1522
% 10    --->  469.5352
% 0     --->  686.2252    |    702.5314   |    710.5230
%           (min = 175.6037)    

% [max_overhoot (Mp=0.1%) = 396]

