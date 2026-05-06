%% ESTIMATION PARAMETER Bb & k
%clear all
clc

% Estimation of w_n and delta
load ('./../resonant_params.mat','mld'); % motor and load parameters
wn = sqrt(mld.k/mld.Jb);
delta = mld.Bb/(2*sqrt(mld.Jb*mld.k));
w = wn*sqrt(1-delta^2);
Ts = 0.001;

tk_vector = [];
thd_k_vector = [];
current_max = 0;
thd_abs = abs(out.thd.signals.values);
t = out.thd.time;
for i = thd_abs
    if i>current_max
        current_max = i; 
    elseif i <= 10e-6 
        thd_k_vector = [thd_k_vector;current_max];
        index_max = find(thd_abs==current_max);
        current_t = t(index_max); 
        tk_vector = [tk_vector; current_t];
        current_max = 0;
    end
        
end



t2 = linspace(0,10,100);
a=sin(3*t2);
figure
plot(t2, a)

%tk = linspace




%Bb_est=mld.Jb*(2*delta_est*w_n_est);
%k_est=mld.Jb*w_n_est^2;
