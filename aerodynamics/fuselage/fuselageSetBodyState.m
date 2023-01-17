function fuselage = fuselageSetBodyState( fuselage, alpha, beta, V, omega ) %#codegen
%fuselageSetBodyState set state.body struct in fuselage struct
%   This function copies the body state variables into the fuselage struct.
%
% Inputs:
% 	fuselage        fuselage struct (see fuselageInit)
% 	alpha       	angle of attack (scalar), in rad
% 	beta            sideslip angle (scalar), in rad
%  	V           	airspeed (scalar), in m/s
% 	omega        	angular velocity (3x1 array), in rad/s
% 	rho         	air density (scalar), in kg/m^3
%
% Outputs:
% 	fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageBodyStateInit, fuselageInit, fuselageCreate, fuselageSetState
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% airspeed
fuselage.state.body.V(:) = V;

% angular velocity
fuselage.state.body.omega(:) = omega;

% angle of attack
fuselage.state.body.alpha(:) = alpha;
% sideslip angle
fuselage.state.body.beta(:) = beta;

end