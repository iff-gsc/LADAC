function fuselageBodyState = fuselageBodyStateInit( ) %#codegen
% fuselageBodyStateInit define and initialize fuselage body state struct.
%
% Syntax:
%   fuselageBodyState = fuselageBodyStateInit( )
% 
% Outputs:
% 	fuselageBodyState    	fuselage body state struct as defined by this
%                           function
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% airspeed, in m/s
fuselageBodyState.V = 0;

% angular velocity of the fuselage frame relative to the earth, in rad/s
fuselageBodyState.omega = zeros( 3, 1 );

% angle of attack, in rad
fuselageBodyState.alpha = 0;
% sideslip angle, in rad
fuselageBodyState.beta = 0;

end