function state = tornadoCreateState( alpha, beta, V_A, omega_Kb, h, is_pgcorr )
% tornadoCreateState create state struct as used by Tornado [1]
% 
% Inputs:
%   alpha       angle of attack (scalar), in rad
%   beta        sideslip angle (scalar), in rad
%   V_A         airspeed (scalar), in m/s
%   omega_Kb    angular velocity of body-fixed frame w.r.t. inertial frame
%               in body-fixed frame (3x1 array), in rad/s
%   h           altitude above mean sea level (scalar), in m
%   is_pgcorr   should Prandtl-Glauert correction be applied to account for
%               compressible flow? (boolean), true: compressible; false:
%               incompressible
% 
% Outputs:
%   state       state struct as used by Tornado
% 
% Literature:
%   [1] User's Guide Tornado 1.0, Release 2.3 2001-01-21,
%       http://tornado.redhammer.se/images/manual.pdf
% 
% See also:
%   TornadoCreateGeo

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

state.alpha = alpha;
state.betha = beta;
state.P = omega_Kb(1);
state.Q = omega_Kb(2);
state.R = omega_Kb(3);
state.AS = V_A;
state.ALT = h;

atmosp = isAtmosphere(h);

state.rho = atmosp.rho;

state.pgcorr = is_pgcorr;

end
