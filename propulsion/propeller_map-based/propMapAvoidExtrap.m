function [ RPM, V ] = propMapAvoidExtrap( prop_fit, RPM, V )
% PROPMAPAVOIDEXTRAP limit RPM and V to the max/min values within the map
%   If RPM or V exceed the maximum or minimum data points in the propeller
%   map, the values of RPM or V are set to the maximum or minimum value.
% 
% Syntax:
%   [ RPM, V ] = propMapAvoidExtrap( prop_fit, RPM, V )
% 
% Inputs:
%   prop_fit                propeller map fit (struct) as defined by the
%                           function propMapFitCreate
%   RPM                     propeller rotational speed (NxM array), in rpm
%   V                       airspeed (NxM array), in m/s
% 
% Outputs:
%   RPM                     propeller rotational speed limited to the 
%                           minimum and maximum values (NxM array), in rpm
%   V                       airspeed limited to the minimum and maximum
%                           values (NxM array), in m/s
% 
% See also:
%   PROPMAPFITCREATE, PROPMAPFITPLOT, PROPMAPFITGETZ, PROPMAPFITGETZDERIV

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

extrap_struct = propMapGetExtrap( prop_fit, RPM, V );

RPM(extrap_struct.is_RPM_max_exceed) = prop_fit.boarders.RPM_max;
RPM(extrap_struct.is_RPM_min_exceed) = prop_fit.boarders.RPM_min;

V(extrap_struct.is_V_max_exceed) = extrap_struct.V_max(extrap_struct.is_V_max_exceed);
V(extrap_struct.is_V_min_exceed) = extrap_struct.V_min(extrap_struct.is_V_min_exceed);

end

function extrap_struct = propMapGetExtrap( prop_fit, RPM, V )

extrap_struct.is_RPM_max_exceed = RPM > prop_fit.boarders.RPM_max;
extrap_struct.is_RPM_min_exceed = RPM < prop_fit.boarders.RPM_min;

RPM(RPM>prop_fit.boarders.RPM_max) = prop_fit.boarders.RPM_max;
RPM(RPM<prop_fit.boarders.RPM_min) = prop_fit.boarders.RPM_min;

V_max = interp1( prop_fit.boarders.RPM, prop_fit.boarders.V_max, RPM );
V_min = interp1( prop_fit.boarders.RPM, prop_fit.boarders.V_min, RPM );

extrap_struct.is_V_max_exceed = V > V_max;
extrap_struct.is_V_min_exceed = V < V_min;

extrap_struct.V_max = V_max;
extrap_struct.V_min = V_min;

end