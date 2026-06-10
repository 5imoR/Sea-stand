%% LAB1: POSITION S.S. CONTROL
clear all, clc ,close all

load ('../param.mat'); % motor parameters
load ('../est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)

Jeq = J_eq_est; Beq = B_eq_est; Tausf = tau_sf_est; 

%% PID  IMPROVEMENTS
PID Parameters
load ('../PID_parameters.mat') % Kp, Ki, Kd and Tl (real derivative)

% pid specs
Mp = 0.1;
ts = 0.15;
% AntiWindup and Feedforward Parameters 
Tw = ts/5; 
Kw = 1/Tw; % anti-windup gain

In_comp = (gbox.N*Req*Jeq)/(drv.dcgain*mot.Kt); % inertia compensation
om_comp = (gbox.N^2* Beq); 
Fri_comp = (Req)/(drv.dcgain*mot.Kt*gbox.N); % friction compensation
BEMF_comp = (gbox.N*mot.Ke)/drv.dcgain; % BEMF compensation


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
lambda_e = [sigma + 1i*omega_d; sigma - 1i*omega_d; sigma]; 

% state feedback matrix for agumented ss
K_e = acker(A_e, B_e, lambda_e);

KI = K_e(1);    % "integrator" fb matrix
K_g = K_e(2:3); % state fb matrix


%% ROBUST TRACKING: ERROR SPACE
%Input Sinusoidal Parameters
Tr = 0.5; % period of the sinusoidal [s]
w0 = 2*pi/Tr; % [rad/s]
Errore Space Model and gain K_z

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


%% ROBUST TRACKING: EXTENDED ESTIMATOR
%Extended Errore Space Model and gain K_z
A_p = [0, 1, 0;
       0, 0, 1; 
       0, -w0^2, 0];
C_p = [1, 0, 0];

% extended ssm (plant model+exo-system)
A_ex = [A_p zeros(3,2);
        B*C_p A];
C_ex = [0 0 0 C];

B_ex = [0;
        0;
        0;
        0;
        -1/Tm];

lambda_ex1 = 2*w_n*exp(1j*(-pi+(pi/3)));
lambda_ex2 = conj(lambda_ex1);
lambda_ex3 = 2*w_n*exp(1j*(-pi+(pi/6)));
lambda_ex4 = conj(lambda_ex3);
lambda_ex5 = -2*w_n;
lambda_ex = [lambda_ex1, lambda_ex2, lambda_ex3, lambda_ex4, lambda_ex5];
K_ex = acker(A_ex', C_ex', lambda_ex);
L_ex = K_ex';
A_a=A_ex-L_ex*C_ex;
B_a=[B_ex, L_ex];
