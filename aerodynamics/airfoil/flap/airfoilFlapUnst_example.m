% This script computes the step response of the unsteady lift coefficient
% due to a step in the flap deflection according to [1].
% Note that currently only the circulatory part of the lift is considered.
% The noncirculatory part could be added in the future if needed.
% 
% Literature:
%   [1] Leishman, J. G. (1994). Unsteady lift of a flapped airfoil by
%       indicial concepts. Journal of Aircraft, 31(2), 288-297.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% flap length relative to the chord (multiple at once)
c_rel = [1,0.25,0.5];

% Mach number
Ma = 0.5;

% airspeed, in m/s
V = Ma * 300;

% airfoil chord, in m
c = 2;

% sample time, in s
T_s = 1e-3;

% simulation time, in s
t_sim = 0.133;

% init unsteady state variable
z = zeros(2,3);
% init lift coefficient
num_samples = t_sim/T_s+1;
c_L_c = zeros(1,3,num_samples);

% init last flap deflection
delta_last = 0;

% compute geometric flap parameters
[F_10,F_11] = airfoilFlapEffectiveness(c_rel);

ones_crel = ones(size(c_rel));

for i = 1:num_samples
    
    % flap deflection, in rad
    delta = 0.1;
    
    % flap deflection rate, in rad/s (numerically)
    delta_dt = (delta-delta_last)/T_s;
    delta_last = delta;
    
    % quasisteady equivalent angle of attack due to flap deflection
    delta_qs = airfoilFlapDeltaQs(F_10,F_11,V,c,delta,delta_dt);
    
    % compute unsteady lift (only circulatory part) and time derivative of
    % the state variable z
    [c_L_c(1,:,i),z_dt] = airfoilFlapUnstLift(V*ones_crel,Ma*ones_crel,c*ones_crel,delta_qs,z);
    
    % Euler forward integration
    z = z + z_dt*T_s;
    
end

%% plot results

plot(2*V*(0:T_s:t_sim)/c,squeeze(c_L_c(1,1,:))/0.1)
hold on
plot(2*V*(0:T_s:t_sim)/c,squeeze(c_L_c(1,2,:))/0.1)
plot(2*V*(0:T_s:t_sim)/c,squeeze(c_L_c(1,3,:))/0.1)
grid on
xlabel('Aerodynamic time, S=2Vt/c')
ylabel('Normal force coefficient / rad.')
legend('airfoil','25% flap','50% flap','Location','SouthEast')
title(["Indicial lift due to flap deflection to be compared with [1], Fig. 4.","Note that the noncirculatory part is neglected here (initial (high frequency) response is different)."])
    