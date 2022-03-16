function local_inflow = fuselageLocalInflowInit( n_segments )
% fuselageLocalInflowInit define and initialize fuselage local inflow
% states struct.
% 
% Inputs:
%   n_segments      number of segments for discretization
% 
% Outputs:
%   local_inflow    fuselage local inflow state struct (as defined by this
%                   function)
% 
% See also:
%   fuselageInit, fuselageStateInit
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% airspeed at control point at the center line in body frame, in m/s
local_inflow.V      = zeros( 3, n_segments );
% local angle of attack at each border at the center line, in rad
local_inflow.alpha  = zeros( 1, n_segments + 1 );
% local sideslip angle at each border at the center line, in rad
local_inflow.beta   = zeros( 1, n_segments + 1 );

end
