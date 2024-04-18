function u = motorStaticSpeed2u( K_T, R, V_bat, d, omega )
% MOTORSTATICSPEED2U calculates the motor setpoint u for a motor with
%   propeller in steady state.
% 
%   The calculation is based on the motor model in [1, eq. 12].
%   For the propeller, a quadratic torque dependency on the motor speed
%   omega is assumed.
% 
% Inputs:
%   K_T     Motor torque constant, N.m/A
%   R       Motor internal resistance, Ohm
%   V_bat   Battery voltage, V
%   d       Propeller drag constant, N.m/rad^2
%            drag constant depending on the motor speed omega, so that
%            torque = d * omega^2
%   omega   Motor speed, rad
% 
% Outputs:
%   u       Motor input [0,1]
%            u scales the battery voltage V_bat, so that
%            voltage = u * V_bat
% 
% Literature:
%   [1] S. Bouabdallah, A. Noth und R. Siegwart. (2004).
%       PID vs LQ control techniques applied to an indoor micro quadrotor.
%       DOI: 10.1109/IROS.2004.1389776
% 
% See also:
%   MOTORSTATICSPEED

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2024 Jonas Withelm
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if d ~= 0
    u = ( K_T^2*omega + R*d*omega.^2 ) / (K_T*V_bat);
else
    u = K_T/V_bat*omega;
end

if u > 1 + 1e-10
    error('There is no solution, higher battery voltage required!')
end
    
end