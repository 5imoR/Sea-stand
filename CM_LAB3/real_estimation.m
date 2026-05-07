%% ESTIMATION PARAMETER Bb & k
%clear all
clc

% Estimation of w_n and delta
load ('./../resonant_params.mat','mld'); % motor and load parameters
wn = sqrt(mld.k/mld.Jb);
delta = mld.Bb/(2*sqrt(mld.Jb*mld.k));
w = wn*sqrt(1-delta^2);
Ts = 0.001;
%%
thd_abs = abs(out.thd.signals.values);
t = out.thd.time;

% tk_vector = [];
% thd_k_vector = [];
% 
%SOL 1
%  for index = 2:length(thd_abs)-1
%     if thd_abs(index)>thd_abs(index-1) && thd_abs(index)>thd_abs(index+1)
%         thd_k_vector = [thd_k_vector thd_abs(index)];
%         tk_vector = [tk_vector t(index)];
%     end 
% end 
% 
% SOL 2
% index = islocalmax(thd_abs);
% index_peaks = find(index == true & thd_abs > 1);
% thd_k_vector = thd_abs(index_peaks);
% tk_vector = t(index_peaks);

% SOL 3 - Signal Processing toolbox needed
delta_tk = pi / w;
minimum_samples = round((0.8 * delta_tk) / Ts); % minimum number of samples between to peaks obtained exploiting eqn (68): tk = k*pi-phi/w so t_k+1-t_k = pi/w 

[thd_k_vector, index_peaks] = findpeaks(thd_abs, 'MinPeakHeight', 1,'MinPeakDistance', minimum_samples); % peaks must be at least of 1 deg and at least after tot samples
tk_vector = t(index_peaks);
%%
M = length(thd_k_vector);
Y = log(thd_k_vector);
phi = [-(0:M-1)',ones(M,1)];

theta_LS = phi\Y;
eta_est = theta_LS(1);

delta_est = eta_est/sqrt(pi^2+eta_est^2);

Tk = tk_vector(2:M) - tk_vector(1:M-1);
wk_est = pi./Tk;

w_est = mean(wk_est);
wn_est = w_est/sqrt(1-delta_est^2);