%% LABORATORIO 2 - DIRECT DIGITAL DESIGN
clear all
clc

%disp('=== SETUP SIMULAZIONE ===')
%tracking_choice = input('Choose architecture (1 = Nominal Tracking, 2 = Robust Tracking): ');
%T = input('Select sample time (0.001, 0.01, 0.05) [s]: ');
% disp('=========================')

step_amplitude = 50;
T=0.05;
tracking_choice=2;

load ('./../../../param.mat'); % motor parameters
load ('./../../../est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)

% Plant
load('./../../../ssPlant_param.mat', 'A','B','C','D', 'lambda', 'w_n', 'lambda_e', 'sigma')
plant_ct = ss(A,B,C,D);
plant_dt = c2d(plant_ct,T,'zoh'); % discretized system
[phi, gamma, H, J] = ssdata(plant_dt);

lambda_dt = exp(lambda*T); % controller poles in z-plan
K = acker(phi,gamma,lambda_dt); % DT controller gain 

N_xu = [phi-eye(2), gamma; H, 0]\[0; 0; 1];
Nx = N_xu(1:2);
Nu = N_xu(3);

% Observer
lambda_o = 5*sigma; 
lambda_o_dt = exp(lambda_o*T); % Observer pole in z-plane

phi11 = phi(1,1); phi22 = phi(2,2); phi12 = phi(1,2); phi21 = phi(2,1);
L = acker(phi22', phi12', lambda_o_dt)'; % Observer gain
% or L = (phi22-lambda_o_dt)/phi12

phi_o = lambda_o_dt;
gamma_o = [gamma(2)-L*gamma(1), (phi22-L*phi12)*L+phi21-L*phi11];
Ho = [0;1];
Jo = [0 1; 0 L];


if tracking_choice == 1 % Nominal control
    Ki = 0; % No integral action
    Nr = Nu + K * Nx; % Feedforward gain

elseif tracking_choice == 2 % robust control
    % extended state space model
    phi_e = [1 H; zeros(2,1) phi];
    gamma_e = [0; gamma];
    lambda_e_dt = exp(lambda_e*T);
    Ke = acker(phi_e, gamma_e, lambda_e_dt);
   
    Ki = Ke(1);
    K = Ke(2:3);
    Nr = Nu + K * Nx; % feedforward gain
end 

% Simulation
% out = sim("digitalDesign_ester.slx"); % CHANGE NAME!!
% disp(max(out.thl));

%% Save results in struct 
filename = 'results_LAB2_2_DirectDigitalDesign.mat';

tracking_names = {'Nominal_Tracking', 'Robust_Tracking'};

% Determine the fields of the structure
current_track = tracking_names{tracking_choice};      
current_T = sprintf('T_%s', strrep(num2str(T), '.', '_')); 

load(filename, 'results');

% Complete the structure with simulation data
sim_data = struct();
sim_data.thl_est = thl_est;
sim_data.thl_meas = thl_meas;
results.(current_track).(current_T) = sim_data;

% Overwrite and save the .mat file 
save(filename, 'results');
