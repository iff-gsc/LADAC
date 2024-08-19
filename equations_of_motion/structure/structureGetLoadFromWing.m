function f = structureGetLoadFromWing(structure,wing,T)

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

XYZ_i_b = wingGetLocalForce( wing );
M_i_b = wingGetLocalProfileMoment( wing );

f_wing = reshape( [XYZ_i_b;M_i_b],[],1 );

T_sc = wing.aeroelasticity.T_sc;

f = T' * T_sc * f_wing;

end