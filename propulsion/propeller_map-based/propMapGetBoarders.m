function prop_map_boarders = propMapGetBoarders( prop_map_scatter )
% PROPMAPGETBOARDERS get information to detect propeller map extrapolation
%   The struct contains the minimum and maximum RPM as well as minimum and
%   maximum airspeeds for each RPM.
% 
% Syntax:
%   prop_map_boarders = propMapGetBoarders( prop_map_scatter )
% 
% Inputs:
%   prop_map_scatter    propeller map scatter (struct) as defined by
%                       propMapScatterCreate
% 
% Outputs:
%   prop_map_boarders   propeller map boarders info as defined by this
%                       function (struct)
% 
% See also:
%   PROPMAPSCATTERCREATE, PROPMAPSCATTERPLOT, PROPMAPAVOIDEXTRAP

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

[RPM,idx_RPM] = unique( prop_map_scatter.RPM );

num_RPM = length( RPM );

RPM_min = min( RPM );
RPM_max = max( RPM );

V_min = zeros( 1, num_RPM );
V_max = zeros( 1, num_RPM );

for i = 1:num_RPM-1
    V_at_RPM = prop_map_scatter.V(idx_RPM(i):idx_RPM(i+1)-1);
    V_min(i) = min( V_at_RPM );
    V_max(i) = max( V_at_RPM );
end

V_at_RPM = prop_map_scatter.V(idx_RPM(end):length(prop_map_scatter.RPM));

V_min(end) = min( V_at_RPM );
V_max(end) = max( V_at_RPM );

prop_map_boarders.RPM = RPM;
prop_map_boarders.V_min = V_min;
prop_map_boarders.V_max = V_max;
prop_map_boarders.RPM_min = RPM_min;
prop_map_boarders.RPM_max = RPM_max;

end