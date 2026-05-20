%%% LABORATORIO 4  LOGITUDINAL STATE SPACE CONTROL OF THE BALANCING ROBOT
clear all
clc

load ('./../balrob_params.mat'); % robot parameters
%load ('./../param.mat','pulse2deg'); 
sens.mpu.gyro.noisevar = sens.mpu.gyro.noisestd^2;
ua2tau1 = 2*gbox.N*mot.Kt/mot.R * [1; -1];

M11 = 2*wheel.Iyy + 2*gbox.N^2*mot.rot.Iyy + (body.m + 2*wheel.m + 2*mot.rot.m)*wheel.r^2;
M21_a = 2*gbox.N*(1-gbox.N)*mot.rot.Iyy; 
M21_b = (body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*wheel.r;
M12_a = M21_a; M12_b = M21_b;
M22 = body.Iyy + 2*(1-gbox.N)^2*mot.rot.Iyy + body.m*body.zb^2 + 2*mot.rot.m*mot.rot.zb^2;
g2 = -(body.m*body.zb + 2*mot.rot.m*mot.rot.zb)*g;

M = [M11 M12_a+M12_b; M21_a+M21_b M22];
G = [0 0 ; 0 g2];

Fv = [2*(gbox.B+wheel.B) -2*gbox.B; -2*gbox.B 2*gbox.B];
Fv1 = Fv + 2*gbox.N^2*mot.Kt*mot.Ke/mot.R *[1 -1; -1 1];

C12 = -(body.m*body.zb+2*mot.rot.m*mot.rot.zb)*wheel.r;


%% Observer 
% Low Pass Filter (LPF)
fc = 0.35;                  
Tc = 1 / (2 * pi * fc);     

num_LPF = Ts;
den_LPF = [Tc, (Ts - Tc)];

%% Controller
% Continuous time state-space model
A = [zeros(2,2) eye(2); -M\G -M\Fv1]; 
B = 2*gbox.N*mot.Kt/mot.R *[zeros(2,2); inv(M)]*[1; -1];
C = [1 0 0 0];
sys_c = ss(A,B,C,0);

% Discretized state-space model (using exact discretization method)
sys_d = c2d(sys_c, Ts, 'zoh');
phi = sys_d.A;
gamma = sys_d.B;
H = sys_d.C;

S = [phi-eye(4) gamma; H 0];
N = S\[0; 0; 0; 0; 1];
Nx = N(1:4); % state feedback matrix
Nu = N(5);   % input feedback matrix

gamma_bar = pi/18; 
theta_bar = pi/360;
u_bar = 1;
Q = diag([1/gamma_bar^2, 1/theta_bar^2, 0 , 0]);
r = 1/u_bar^2;
rho = 500; % or 500

tracking = 1; % 1 = Nominal tracking 2 = Robust tracking

if tracking ==1
    K_lqr = dlqr(phi,gamma,Q,rho*r);
    Ki = 0;
else
    % extended state space model
    phi_e = [1 H; zeros(4,1) phi];
    gamma_e = [0; gamma];
    q11 = 0.1; % or 1 , by trial and error
    Q_e = diag([1/gamma_bar^2, 1/theta_bar^2, 0 , 0, q11]);
    K_e = dlqr(phi_e,gamma_e,Q_e,rho*r);
    Ki = K_e(1);         
    K_lqr= K_e(2:5);
end 

%% Simulation
scenario_selection = 2;
if scenario_selection == 1
    
    % first scenario
    theta0_deg = 5;
    gam_ref = 0;
    u_dist = 0;
    t_dist = 0;

elseif scenario_selection == 2    
    % second scenario
    theta0_deg = 0;
    positional_displacement = 0.01; % m
    gam_ref = (positional_displacement/wheel.r)*rad2deg;
    u_dist = 0;
    t_dist = 0;

elseif scenario_selection == 3
    % third scenario
    theta0_deg = 0;
    positional_displacement = 0.01; % m
    gam_ref = (positional_displacement/wheel.r)*rad2deg;
    
    t_dist = 10;
    u_dist = 5/drv.duty2V;
end 

q0_sim = [0; theta0_deg*deg2rad];
dot_q0_sim = [0;0];

out = sim("plant4_ester");

