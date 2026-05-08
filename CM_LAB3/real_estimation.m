%% ESTIMATION PARAMETER Bb & k
clear all
clc

load ('./../resonant_params.mat','mld'); % motor and load parameters
load ('./../resonant_params.mat','sens');   
wn = sqrt(mld.k/mld.Jb);
delta = mld.Bb/(2*sqrt(mld.Jb*mld.k));
w = wn*sqrt(1-delta^2);
Ts = 0.001;
t1 = 0.7;
t0 = 0.2;

%%
% Estimation of natural frequency and damping factor
valid_idx = thd.time >= 1.0; 
t = thd.time(valid_idx);
thd_abs = abs(thd.signals.values(valid_idx));

delta_tk = pi / w;
minimum_samples = round((0.8 * delta_tk) / Ts); % minimum number of samples between to peaks obtained exploiting eqn (68): tk = k*pi-phi/w so t_k+1-t_k = pi/w 

% Find peaks (absolute values) and corresponding time instances 
[thd_k_vector, index_peaks] = findpeaks(thd_abs, 'MinPeakHeight', 1,'MinPeakDistance', minimum_samples); % peaks must be at least of 1 deg and at least after tot samples
tk_vector = t(index_peaks); 

% Linear LS fitting of data (log)
M = length(thd_k_vector);       % number of peaks
Y = log(thd_k_vector(:));       % column vector of log(abs(peak_value))  (Y = phi*theta)
phi = [-(0:M-1)', ones(M,1)];   % regressor matrix
theta_LS = phi\Y;               % LS solution (slope and intercept) 

eta_est = theta_LS(1);                      % estimate of the loarithmic decrement
delta_est = eta_est/sqrt(pi^2+eta_est^2);   % estimate of the damping factor
Tk = tk_vector(2:M) - tk_vector(1:M-1);     % vector of measured time intervals between consecutive peaks 
wk_est = pi./Tk;                            % vector of estimates of the frequency for every time interval Tk 
w_est = mean(wk_est);                       % averaged estimate of the frequency 
wn_est = w_est/sqrt(1-delta_est^2);         % estimate of the natural frequency

% Estimation of beam viscous friction coefficient and elastic joint stiffness
Bb_est = mld.Jb*2*delta_est*wn_est;   % estimate of the beam friction coefficient
k_est = mld.Jb*wn_est^2;              % estimate of the elastic joint stiffness

%%
save('est_param_resonant.mat', 'Bb_est', 'k_est')