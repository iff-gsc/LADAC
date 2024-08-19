function omega = motorStaticSpeed( K_T, R, V_bat, d, u )
% MOTORSTATICSPEED calculates the motor speed omega for a motor with
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
%   u       Motor input [0,1]
%            u scales the battery voltage V_bat, so that
%            voltage = u * V_bat
% 
% Outputs:
%   omega   Motor speed, rad
% 
% Literature:
%   [1] S. Bouabdallah, A. Noth und R. Siegwart. (2004).
%       PID vs LQ control techniques applied to an indoor micro quadrotor.
%       DOI: 10.1109/IROS.2004.1389776
% 
% See also:
%   MOTORSTATICSPEED2U

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if d ~= 0
    omega = ( sqrt( 4*d*K_T*R*V_bat*u + K_T^4) - K_T^2 ) / ( 2*d*R );
else
    omega = V_bat/K_T*u;
end

end