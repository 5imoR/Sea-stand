%%% LABORATORIO 4  LOGITUDINAL STATE SPACE CONTROL OF THE BALANCING ROBOT
clear all
clc

load ('./../balrob_params.mat'); % robot parameters
load ('./../param.mat','pulse2deg'); % il prof non l'ha messo nella lista sopra
sens.mpu.gyro.noisevar=sens.mpu.gyro.noisestd^2;
%% Lagrangian Dynamic Equation: matrix formulation
% M(q)
M11 = 2*wheel.Iyy + 2*gbox.N^2*mot.rot.Iyy+(body.m+2*wheel.m+2*mot.rot.m)*wheel.r^2;
M21_1 = 2*gbox.N*(1-gbox.N)*mot.rot.Iyy;
M21_2 = (body.m*body.zb+2*mot.rot.m*mot.rot.zb)*wheel.r;
M22 = body.Iyy+2*(1-gbox.N)^2*mot.rot.Iyy+body.m*(body.zb^2)+2*mot.rot.m*(mot.rot.zb^2);
% C(q,dot_q)
C11 = 0;
C12 = -(body.m*body.zb+2*mot.rot.m*mot.rot.zb)*wheel.r;
% Fv
F11 = 2*(gbox.B+wheel.B);
F12 = -2*gbox.B;
F22 = 2*gbox.B;
F=[F11 F12; F12 F22];
% g(q)
G1 = 0;
G2 = -(body.m*body.zb+2*mot.rot.m*mot.rot.zb)*g;

%% State-Space Controller: lqr method with bryson'rule
% Filters Params (for the simple state observer)
fc = 0.32;
Tc = 1/(2*pi*fc);
C = Ts/(Tc+Ts);
% Composition discrete state space model
M21 = M21_1+M21_2;
M = [M11,M21;M21,M22];
G = [0 0; 0 G2];
Fv = F + ((2*gbox.N^2*mot.Kt*mot.Ke)/(mot.R))*[1 -1;-1 1];
A = [zeros(2,2) eye(2,2);
    -inv(M)*G -inv(M)*Fv];
B = (2*gbox.N*mot.Kt/mot.R)*[zeros(2,2); inv(M)]*[1;-1];
sysG=ss(A,B,[1 0 0 0],0);
% Discretize the system for simulation
sysD = c2d(sysG, Ts, 'zoh');
[Phi,Gamma,H] = ssdata(sysD);
% feedforward matrices (discrete case)
E = [Phi-eye(4) Gamma; H 0];
N = E\[0; 0; 0; 0; 1];
Nx = N(1:4); 
Nu = N(5);   
% Define the LQR controller gains
Q = [1/(pi/18)^2    0          0   0;
      0          1/(pi/360)^2  0   0
      0             0          0   0
      0             0          0   0]; % State weighting matrix
p = 5000;
R = p*1; % Control weighting matrix
K = dlqr(Phi, Gamma, Q, R); 

%Integral Action
Phi_e = [1 H; zeros(4,1) Phi];
Gamma_e = [0; Gamma];
Q_e=[ 1       0           0        0  0;
      0   1/(pi/18)^2     0          0  0; 
      0       0       1/(pi/360)^2   0  0;   
      0       0           0          0  0;
      0       0           0          0  0];
K_e = dlqr(Phi_e,Gamma_e,Q_e,R);
