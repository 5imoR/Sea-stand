
% Plot SRL to analyze how closed-loop eigenvalues move as input cost (r) decreases
Mp = 0.3;   % Maximum overshoot (30%)
ts = 0.85;
 delta = (log(1/Mp))/sqrt(pi^2+(log(1/Mp))^2);
phi = atan(sqrt(1-delta^2)/(delta));
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