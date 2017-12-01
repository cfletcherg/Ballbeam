% parameters are separated into two files -- one that are specific for the
% homework set (here) and the other that contains values that won't change.
% These can be combined into one, but be sure to track homework specific 
% parameters carefully.

% inverted ballbeam - parameter file for hw8
ballbeamParam % general ballbeam parameters

% tuning parameters
% this could include your damping ration for inner/outer loop, desired rise
% times, separation value between inner loop and outer, et.

% saturation limits for beam angle and total force

% PD design for inner loop
% calculate the kp and kd gains for theta here...
Tr_theta = .15;
wn_theta = 2.2/Tr_theta;
zeta = .707;
ze = P.length/2;
kdp_theta_denominator = P.length/(ze^2*P.m1+P.m2*(P.length^2/3));

P.kp_th  = (wn_theta)^2/(kdp_theta_denominator); % kp - inner
P.kd_th  = (2*zeta*wn_theta)/kdp_theta_denominator; % kd - inner

% DC gain for inner loop
k_DC_th = 1;

% integrator pole
integrator_pole = -5;

% PD design for outer loop
% calculate the kp and kd gains for theta here...
wn_z = wn_theta/10;

P.kp_z   = wn_z^2/-P.g; % kp - outer
P.kd_z   = 2*zeta*wn_z/-P.g; % kd - outer
P.ki_z   = -.5;

% Saturation limits for beam angle and total force
P.theta_max = 20;
P.Fmax = 20;

%--------------------------
% state space design
P.A = [0 0 1 0;...
     0 0 0 1;...
     0 -P.g 0 0;...
     (P.m1)/((P.m2*P.length^2)/3+P.m1*ze^2) 0 0 0];
P.B = [0;...
     0;...
     0;...
     P.length/((P.m2*P.length^2)/3+P.m1*ze^2)];
P.C = [1 0 0 0];

A1 = [P.A, zeros(4,1); -C, 0];
B1 = [P.B; 0];

% gains for pole locations
des_char_poly = conv(conv(...
    [1,2*zeta*wn_z,wn_z^2],...
    [1,2*zeta*wn_theta,wn_theta^2]),...
    poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
rank(ctrb(A,B));

K1 = place(A1,B1,des_poles);
P.K = K1(1:4);
P.ki = K1(5);

% observer design
des_obsv_char_poly = conv(...
    [1,2*zeta*wn_z,wn_z^2],...
    [1,2*zeta*wn_theta,wn_theta^2]);
des_obsv_poles = roots(des_obsv_char_poly);

rank(obsv(P.A,P.C));
P.L = place(P.A',P.C', des_obsv_poles)';
