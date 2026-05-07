%% LABORATORY 3 - RESONANT LOAD POSITION CONTROLLER
clear all
clc

%% LOAD PARAMETERS
load ('./../resonant_params.mat');                      % Nominal motor and load parameters
load ('./../est_param.mat','B_eq_est','tau_sf_est');    % Estimated friction parameters 
load ('./../est_param_resonant.mat');                   % Estimated elastic joint parameters

Beq = B_eq_est;
tau_sf = tau_sf_est;
Req = mot.R + sens.curr.Rs;         % Total equivalent resistance
Jeq = mot.J + mld.Jh / (gbox.N^2);  % Equivalent inertia seen at motor side

Ts = 0.001;                         % Sampling time [s]
t1 = 0.7;
t0 = 0.2;

%% CHOICE OF PID 

%% PID CONTROLLER & ANTI-WINDUP
% Specifications for PID design
alpha= 4;   % Ratio between Ti and Td
Mp = 0.3;   % Maximum overshoot (30%)
ts = 0.85;  % Settling time [s]

% Plant transfer function U --> thh
numP= [(drv.dcgain*mot.Kt*mld.Jb)  (drv.dcgain*mot.Kt*Bb_est)  (drv.dcgain*mot.Kt*k_est)];
denP = [(gbox.N*Req*Jeq*mld.Jb)  gbox.N*(Req*Jeq*Bb_est+Req*mld.Jb*Beq+mot.Kt*mot.Ke*mld.Jb) gbox.N*(mot.Kt*mot.Ke*Bb_est+Req*Beq*Bb_est+Req*k_est*(Jeq+mld.Jb/(gbox.N^2))) gbox.N*(mot.Kt*mot.Ke*k_est+Req*k_est*Beq+Req*k_est*Bb_est/(gbox.N^2)) 0];

P_s = tf(numP,denP);

% Compute PID gains, required damping factor, and natural frequency
[Kp, Ki, Kd, delta, w_n] = LAB3_design_controller(P_s, Mp, ts, alpha);

% Real derivative filter design 
Tl = 1 / (10 * w_n);

% AntiWindup parameters
Tw = ts / 5;
Kw = 1 / Tw;    % Anti-windup gain


%% STATE-SPACE CONTROLLER WITH EIGENVAlUES ALLOCATIONS METHOD
% High pass filter for real derivaties (representative of the observer) 
wc = 2*pi*50;         % Cut.off frequency [rad/s]
deltac = 1/sqrt(2);   % Damping factor

% State space model of the plant (State vector x' = [th_h, th_d, w_h, w_d]')
A = [0     0                                 1                                              0; 
     0     0                                 0                                              1; 
     0     (k_est/(gbox.N^2*Jeq))            -(1/Jeq)*(Beq+((mot.Ke*mot.Kt)/Req))             0; 
     0     -k_est/mld.Jb-k_est/(Jeq*gbox.N^2)  -(Bb_est/mld.Jb)+(1/Jeq)*(Beq+mot.Ke*mot.Kt/Req) -Bb_est/mld.Jb];

B = [0; 0; (mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req); -(mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req)];

Bd=[0; 0; 1/((gbox.N^2)*Jeq); -1/((gbox.N^2)*Jeq)];

C=[1 0 0 0];

% Target poles for the closed-loop system
phi = atan(sqrt(1-delta^2)/(delta));
p1 = w_n*exp(1i*(-pi+(phi)));     % First pair of dominant poles
p2 = conj(p1);
p3 = w_n*exp(1i*(-pi+(phi/2)));   % Second pair of faster poles
p4 = conj(p3);
poles = [p1, p2, p3, p4];         % Poles vector

% State feedback matrix
K = place(A, B, poles);  

% Feedforward matrices
S = [A B; C 0];
N = S \ [0; 0; 0; 0; 1];
Nx = N(1:4);    % state feedback matrix
Nu = N(5);      % input feedback matrix

%% STATE SPACE CONTROLLER WITH LQR METHOD
sysG=ss(A,B,C,0);
sysGp=ss(-A,-B,C,0);

% -- LQR via Symmetric Root Locus -- 
% Plot SRL to analyze how closed-loop eigenvalues move as input cost (r) decreases
figure(1)
rlocus(sysG*sysGp)
hold on
% Plot performance boundaries based on specs (Settling time and Overshoot)
plot([-3/ts -3/ts], [-1 1]*(-3/ts*tan(phi)), '--', 'Color', [0 0.4470 0.7410])
plot([-100 -3/ts], [-100 -3/ts]*tan(phi), '--', 'Color', [0 0.4470 0.7410])
plot([-100 -3/ts], [-100 -3/ts]*-tan(phi), '--', 'Color', [0 0.4470 0.7410])
grid on
axis([-60 60 -60 60]);
title('Symmetric Root Locus (LQR Design)')
hold off

% Gain 1/r chosen graphically to keep poles inside the desired performance region
r = 1/4.59e3;

% Compute LQR gain matrix (Standard cost function with Q = C'*C)
K_lqr=lqr(sysG, C'*C, r);

% -- LQR via Bryson's Rule (Cost Weights tuning) ---
Q = [1/(0.3*50*pi/180)^2 0           0  0;
     0                   1/(pi/36)^2 0  0;
     0                   0           0  0;
     0                   0           0  0;];
R = 1/100;

% Compute LQR gain matrix using Bryson's Rule weights
K_lqr2 = lqr(sysG,Q,R);