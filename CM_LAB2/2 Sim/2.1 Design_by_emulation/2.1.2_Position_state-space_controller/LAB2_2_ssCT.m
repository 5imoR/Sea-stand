%% LABORATORIO 2 - CT STATE SPACE CONTROL 
clear all, clc, close all

load ('../param.mat'); % motor parameters
load ('../est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)
Jeq = J_eq_est; Beq = B_eq_est; Tausf = tau_sf_est; 

sampleTime_options = [0.001 0.01 0.05]; % sampling time [s] 
T = sampleTime_options(1);

% Choose architecture: 1= Nominal Tracking, 2=Robust Tracking
tracking_choice = 2;

%% PLANT 
load('../ssPlant_param.mat') % "A","B","C","D","delta","w_n","sigma","omega_d","K", "Nr", K2","Ki","Nr2"

%% OBSERVER 
% Pole
lambda_o = -5 * w_n; 

% Observer gain
L = -(1/Tm) - lambda_o;

% State space matrices in the CT case
Ao = -(1/Tm) - L; 
Bo = [Km/(gbox.N*Tm), (-(1/Tm) - L)*L];
Co = [0; 1];
Do = [0, 1; 0, L];

if tracking_choice == 1
    Ki = 0; 

elseif tracking_choice == 2
    K = K2;
    Nr = Nr2;
end 

out = sim("ss2CT_ester.slx");
