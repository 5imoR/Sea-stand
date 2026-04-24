%% LABORATORIO 2 - CT STATE SPACE CONTROL 
clear all
clc


%tracking_choice = input('Choose architecture (1 = Nominal Tracking, 2 = Robust Tracking): ');
%method_choice = input('Choose discretization method (1 = B. Euler, 2 = F. Euler, 3 = Tustin): ');
%step_amplitude = input('Select step amplitude (es. 40, 70, 120): ');
%T = input('Select sample time (0.001, 0.01, 0.05) [s]: ');


load ('./../../../../param.mat'); % motor parameters
load ('./../../../../est_param.mat'); % estimated parameters (J_eq, B_eq, tau_sf)
load ('./../../../../PID_parameters.mat') % Kp, Ki, Kd and Tl (real derivative)

method_choice = 2;
step_amplitude = 50;
T = 0.05;
tracking_choice = 2;

% sampleTime_options = [0.001 0.01 0.05]; % sampling time [s] 
% T = sampleTime_options(3);
% % Choose architecture: 1= Nominal Tracking, 2=Robust Tracking

% % Choose discretization method: 1 = FE, 2 = BE, 3 = Tustin
% method_choice = 1;

% PLANT 
load('./../../../../ssPlant_param.mat')

% OBSERVER 
% Pole
lambda_o = 5 * sigma; 

% Observer gain
L = -(1/Tm) - lambda_o;

% State space matrices in the CT case
Ao = -(1/Tm) - L; 
Bo = [Km/(gbox.N*Tm), (-(1/Tm) - L)*L];
Co = [0; 1];
Do = [0, 1; 0, L];

% State space matrices in the DT case
if method_choice == 2 % Forward Euler
    phi_o = 1 + Ao*T;
    gamma_o = Bo*T;
    Ho = Co;
    Jo = Do;
elseif method_choice == 1 % Backward Euler
    phi_o = 1/(1-Ao*T);
    gamma_o = phi_o*Bo*T;
    Ho = Co*phi_o;
    Jo = Do+Co*phi_o*Bo*T;
elseif method_choice == 3 % Tustin
    sysCd = c2d(ss(Ao,Bo,Co,Do),T,'tustin');
    [phi_o, gamma_o, Ho, Jo] = ssdata(sysCd);
end 


if tracking_choice == 1
    Ki = 0; 
elseif tracking_choice == 2
    K = K2;
    Nr = Nr2;
end 

% out = sim("ss2DT_ester.slx"); % CHANGE NAME

%% Save results in struct 
filename = 'results_LAB2_2_StateSpace.mat';

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
