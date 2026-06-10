%% LABORATORIO 4  LOGITUDINAL STATE SPACE CONTROL OF THE BALANCING ROBOT

% -------------- ISTRUZIONI !!!! -----------------------------------------
% 1. Runnare primo blocco (1) con Run Section
% 2. Selezionare il tipo di test (1 o 2)
% 3. Runnare secondo blocco (2) per completare workspace (NON VA MODIFICATO
% NEJITNE IN TEORIA)
% 4. Runnare simulazione
% 5. SOLO ALLA FINE RUNNARE LA PARTE DI SALVATAGGIO DATI (3) se si vogliono
    % salvare , attento che sovrascrive...

%% 1. Clearing and loading parameters
clear all
clc

load ('./../balrob_params.mat'); % robot parameters 
sens.mpu.gyro.noisevar = sens.mpu.gyro.noisestd^2;

%% 2. Tracking selection and 'controller computation'
% 1 = Nominal tracking 
% 2 = Robust tracking 

test_number = 1;

% Lagrangian Dynamic Equation: matrix formulation
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

% State-Space Controller: lqr method with bryson'rule
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

switch test_number

    case 1 % Nominal tracking
        K = K;
        Ki = 0;
        
        field_name = 'Nominal_tracking';

    case 2 % Robust tracking
        % Integral Action
        Phi_e = [1 H; zeros(4,1) Phi];
        Gamma_e = [0; Gamma];
        Q_e=[ 1       0           0        0  0;
              0   1/(pi/18)^2     0          0  0; 
              0       0       1/(pi/360)^2   0  0;   
              0       0           0          0  0;
              0       0           0          0  0];
        K_e = dlqr(Phi_e,Gamma_e,Q_e,R);
        Ki = K_e(1);         
        K = K_e(2:5);

        field_name = 'Robust_tracking';


end 

%% 3. DATA SAVING -- da runnare dopo la simulazione 
if isfile('results_LAB4.mat')
    load('results_LAB4.mat', 'Data_Lab4');
end

Data_Lab4.(field_name).gamma_est = gamma_est;
Data_Lab4.(field_name).th_est = th_est;
Data_Lab4.(field_name).dot_gamma_est = dot_gamma_est;
Data_Lab4.(field_name).dot_th_est = dot_th_est;
Data_Lab4.(field_name).duty = duty;


save('results_LAB4.mat', 'Data_Lab4');

