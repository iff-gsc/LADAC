function p_structure = wingGetStructureLoad( wing )
% wingGetStructureLoad compute structure load vector from wing
% 
% Inputs:
%   wing            wing struct, see wingCreate
% 
% Outputs:
%   p_structure     structure load vector; concentrated forces (X,Y,Z) and
%                   pitching moment (M) at each control point (i) -->
%                   p_wing = (X_1,Y_1,Z_1,M_1,X_2,Y_2,Z_2,M_2,...) -->
%                   convert p_wing to p_structure witch interpolation
%                   matrix: p_structure = T_sc * p_wing
% 
% See also:
%   wingGetLocalForce, wingGetLocalAirfoilMoment
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

XYZ_i_b         = wingGetLocalForce( wing );
M_profile_i_b   = wingGetLocalAirfoilMoment( wing );

p_wing          = reshape([XYZ_i_b;M_profile_i_b],[],1);

p_structure     = wing.aeroelasticity.T_sc * p_wing;

end