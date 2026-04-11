function [Kp, Ki, Kd] = LAB0_design_controller(P_s, Mp, Ts, alpha)
    
    % Compute damping factor and the gain crossover frequency
    delta=(log(1/Mp))/sqrt(pi^2+(log(1/Mp))^2);
    w_gc=3/(delta*Ts); 
    
    % Compute margin of phase     
    phi_m_rad=atan((2*delta)/(sqrt(sqrt(1+4*delta^4)-2*delta^2)));

    % Valutation of plant P(s) at frequency w_gc
    [mag, phase_deg] = bode(P_s, w_gc);
    mag = squeeze(mag);
    phase_rad = squeeze(phase_deg)*pi/180;

    % Compute Delta K (missing gain) e Delta phi (missing phase)
    Delta_K = 1/mag;
    Delta_phi = -pi+phi_m_rad-phase_rad;

    % Compute Kp Ki Kd
    Kp = Delta_K * cos(Delta_phi);
    Td = (tan(Delta_phi) + sqrt(tan(Delta_phi)^2 + 4/alpha)) / (2 * w_gc);
    Ti = alpha * Td;
    Ki = Kp / Ti;
    Kd = Kp * Td;