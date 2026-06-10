%% LAB1: POSITION S.S. CONTROL
clear all, clc ,close all

load ('../param.mat'); % motor parameters
load ('../est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)

Jeq = J_eq_est; Beq = B_eq_est; Tausf = tau_sf_est; 

% STATE SPACE CONTROL 
%% NOMINAL TRACKING

% High Pass Filter
w_c = 2*pi*50;
delta_c = 1/sqrt(2);

% State Space Controller
A = [0 1; 0 -1/Tm];
B = [0; Km/(gbox.N*Tm)];
C = [1 0];
D = 0;

% poles (lambda)
Mp = 0.1;
ts = 0.15;
delta = (log(1/Mp)) / sqrt(pi^2 + (log(1/Mp))^2);
w_n = 3 / (delta * ts); 

sigma = -delta*w_n; % real part
omega_d = w_n*sqrt(1-delta^2); % imaginary part

lambda = [sigma + 1i*omega_d; sigma - 1i*omega_d]; % vector of poles

% state feedback matrix
K = place(A, B, lambda); 

% feedforward matrices
S = [A B; C D];
N = inv(S)*[0; 0; 1];
Nx = N(1:2); % state feedback matrix
Nu = N(3);   % input feedback matrix


%% ROBUST TRACKING: INTEGRAL ACTION

% Agumented state space matrices
A_e = [0 C; zeros(2,1) A];
B_e = [0;B];
C_e = [0 C];

% poles 
lambda_e = [sigma; sigma; sigma]; 

% state feedback matrix for agumented ss
K_e = acker(A_e, B_e, lambda_e);

KI = K_e(1);    % "integrator" fb matrix
K_g = K_e(2:3); % state fb matrix


%% ROBUST TRACKING: ERROR SPACE
%Input Sinusoidal Parameters
Tr = 0.5; % period of the sinusoidal [s]
w0 = 2*pi/Tr; % [rad/s]
%Errore Space Model and gain K_z

% error space matrices 
alfa0 = 0; alfa1 = w0^2; alfa2 = 0;

A_z = [ 0, 1, 0, zeros(1,2);
       0, 0, 1, zeros(1,2);
       -alfa0, -alfa1, -alfa2, C;
       zeros(2,1), zeros(2,1), zeros(2,1), A];
B_z = [0; 0; 0; B];

% poles 
lambda_z1 = w_n * exp(1i * (-pi + pi/4)); 
lambda_z2 = conj(lambda_z1); 
lambda_z3 = w_n * exp(1i * (-pi+pi/6));
lambda_z4 = conj(lambda_z3); 
lambda_z5 = - w_n;

lambda_z = [lambda_z1, lambda_z2, lambda_z3, lambda_z4, lambda_z5];

% feedback matrix
K_z = place(A_z, B_z, lambda_z);

k_z0 = K_z(1); k_z1 = K_z(2); k_z2 = K_z(3); % coefficients of H numerator
k_zx = K_z(4:5); % K_eta


%% Salvataggio dati simulazioni
field_name1 = 'deg120';
field_name2 = 'without_tau'; 

if isfile('results_IA.mat')
    load('results_IA.mat', 'Data_IA');
end

Data_IA.(field_name1).(field_name2).pos = pos;
%Data_Lab1.(field_name1).(field_name2).sat_in = sat_in;
%Data_Lab1.(field_name1).(field_name2).sat_out = sat_out;
Data_IA.(field_name1).(field_name2).track_err = track_err;

save('results_IA.mat', 'Data_IA');