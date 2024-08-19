function coeff_loc = fuselageLocalCoeffInit( n_segments )
% fuselageLocalCoeffInit define and initialize fuselage local aerodynamic
% coefficients struct.
% 
% Syntax:
%   coeff_loc = fuselageLocalCoeffInit( n_segments )
% 
% Inputs:
%   n_segments      number of segments for discretization
% 
% Outputs:
%   coeff_loc       fuselage local aerodynamic coefficient struct (as 
%                   defined by this function)
% 
% See also:
%   fuselageInit, fuselageStateInit

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% airspeed at control point at the center line in body frame, in m/s
coeff_loc.C_XYZ_b_i   	= zeros( 3, n_segments );

end
