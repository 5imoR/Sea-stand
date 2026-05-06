%%% LABORATORIO 3 RESONANT LOAD POSITION CONTROLLER
clear all
clc

%% LOAD PARAMETERS
load ('./../resonant_params.mat'); % motor and load parameters
load ('./../est_param.mat','B_eq_est','tau_sf_est'); % estimated parameters ( B_eq, tau_sf)
%load('./../../../ssPlant_param.mat', 'A','B','C','D')
Beq=B_eq_est;
tau_sf=tau_sf_est;
Req = mot.R + sens.curr.Rs;
Jeq = mot.J + mld.Jh / (gbox.N^2); 
Ts = 0.001; % Sampling time

%% PID CONTROLLER & ANTI-WINDUP
% Specifications for PID design
alpha= 4;
Mp = 0.3;
Tsett = 0.85;
% PID parameters (found using a functon LAB0_design_controller)
% Plant transfer function U --> thh
num = [(drv.dcgain*mot.Kt*mld.Jb)  (drv.dcgain*mot.Kt*mld.Bb)  (drv.dcgain*mot.Kt*mld.k)];
den = [(gbox.N*Req*Jeq*mld.Jb)  gbox.N*(Req*Jeq*mld.Bb+Req*mld.Jb*Beq+mot.Kt*mot.Ke*mld.Jb) gbox.N*(mot.Kt*mot.Ke*mld.Bb+Req*Beq*mld.Bb+Req*mld.k*(Jeq+mld.Jb/(gbox.N^2))) gbox.N*(mot.Kt*mot.Ke*mld.k+Req*mld.k*Beq+Req*mld.k*mld.Bb/(gbox.N^2)) 0];
P_s = tf(num,den);
[Kp, Ki, Kd] = LAB3_design_controller(P_s, Mp, Tsett, alpha);
% Real derivative design 
% Find Tl for "real derivative" (High Pass Filter)
delta = (log(1/Mp)) / sqrt((pi^2) + (log(1/Mp))^2);
w_n = 3 / (delta * Tsett); 
Tl = 1 / (10 * w_n);
% AntiWindup parameters
Tw=Tsett/5;
Kw=1/Tw;

%% STATE-SPACE CONTROLLER WITH EIGEN ALLOCATIONS METHOD
% High pass filter real derivaties 
wc=2*pi*50;
deltac=1/sqrt(2);
% State space model
A=[0     0                                 1                                              0; 
   0     0                                 0                                              1; 
   0     (mld.k/(gbox.N^2*Jeq))            -(1/Jeq)*(Beq+((mot.Ke*mot.Kt)/Req))             0; 
   0     -mld.k/mld.Jb-mld.k/(Jeq*gbox.N^2)  -(mld.Bb/mld.Jb)+(1/Jeq)*(Beq+mot.Ke*mot.Kt/Req) -mld.Bb/mld.Jb];
B=[0;
   0;
   (mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req);
   -(mot.Kt*drv.dcgain)/(gbox.N*Jeq*Req)];
Bd=[0;
   0;
   1/((gbox.N^2)*Jeq);
   -1/((gbox.N^2)*Jeq)];
C=[1 0 0 0];
% poles (lambda)
phi=atan(sqrt(1-delta^2)/(delta));
lambda1 = w_n*exp(1i*(-pi+(phi)));
lambda2 = conj(lambda1);
lambda3 = w_n*exp(1i*(-pi+(phi/2)));
lambda4 = conj(lambda3);
lamb = [lambda1, lambda2, lambda3, lambda4]; % vector of poles
% state feedback matrix
K = place(A, B,lamb); 
% feedforward matrices
S = [A B; C 0];
N = inv(S)*[0; 0; 0; 0; 1];
Nx = N(1:4); % state feedback matrix
Nu = N(5);   % input feedback matrix

%% STATE SPACE CONTROLLER WITH LQR METHOD
sysG=ss(A,B,C,0);
sysGp=ss(-A,-B,C,0);
%figure
%rlocus(sysG*sysGp)
%hold on
%plot([-3/Tsett -3/Tsett], [-1 1]*(-3/Tsett*tan(phi)), '--', 'Color', [0 0.4470 0.7410])
%plot([-100 -3/Tsett], [-100 -3/Tsett]*tan(phi), '--', 'Color', [0 0.4470 0.7410])
%plot([-100 -3/Tsett], [-100 -3/Tsett]*-tan(phi), '--', 'Color', [0 0.4470 0.7410])
%grid on
%hold off
r=1/4.59e3;
K_lqr=lqr(sysG, C'*C, r);

% Another versione of lqr
Q=[1/(0.3*50*pi/180)^2 0           0  0;
   0                   1/(pi/36)^2 0  0;
   0                   0           0  0;
   0                   0           0  0;];
R=1/100;
K_lqr2=lqr(sysG,Q,R);