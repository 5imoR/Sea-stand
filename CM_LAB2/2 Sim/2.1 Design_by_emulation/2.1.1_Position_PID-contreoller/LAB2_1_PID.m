%% LABORATORIO 2 - DESIGN BY EMULATION
clear all, clc, close all

load ('./../../../../param.mat'); % motor parameters
load ('./../../../../est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)
load ('./../../../../PID_parameters.mat') % Kp, Ki, Kd and Tl (real derivative)
Tl = 3/2*Tl;

T = input('Select sample time (0.001, 0.01, 0.05) [s]: ');
arch_choice = input('Choose architecture  (1 = No anti-windup, 2 = With anti-windup): ');
method_choice = input('Choose discretization method (1 = B. Euler, 2 = F. Euler, 3 = Tustin, 4 = Exact): ');

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
sysCd_deriv = c2d(sysC_deriv, T, 'tustin'); 
[N3, D3] = tfdata(sysCd_deriv, 'v');

% exact
sysCd_deriv_e = c2d(sysC_deriv, T, 'zoh'); 
[N4, D4] = tfdata(sysCd_deriv_e, 'v');

Nd_options = [[1, -1]; [1, -1]; N3; N4];
Dd_options = [[Tl+T, -Tl]; [Tl, T-Tl]; D3; D4]; % 1:BE 2:FE 3:tustin 4:Exact
Nd = Nd_options(method_choice,:);
Dd = Dd_options(method_choice,:);

%% Integrator implementation
% exact
sysC_int = tf(1, [1 0]); 
sysCd_int = c2d(sysC_int, T, 'zoh'); 
[Ni_exact, Di_exact] = tfdata(sysCd_int, 'v');

%% simulation 
out = sim("LAB2_1_PID_sim.slx");
disp(max(out.thl))


