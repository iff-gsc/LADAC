function [coeff, airfoil_map] = wingAirfoilMapInterpCoeff( airfoil_map, segment_type_local, type, alpha, Ma, Re, act1, act2  ) %#codegen
% wingAirfoilMapInterpCoeff gets the aerodynamic coefficient from a profile
% map.
%   Since there are big problems with code generation if multiple
%   aerodynamic maps are used in Simulink, the profile_map is an Scell
%   array (see cell2scellArr, mat_file_format.md).
% 
% Inputs:
%   airfoil_map         Scell array containing the profile map as explained
%                       in mat_file_format.md.
%   segment_type_local  Index that specifies the profile (needed for
%                       support of multiple profiles per wing).
%   type                type of coefficient used: 'c_L', 'c_D' or 'c_m'
%   alpha               angle of attack (1xn array)
%   Ma                  Mach number (1xn array)
%   Re                  Reynolds number (1xn array)
%   act1                1st actuator state (1xn array)
%   act2                2nd actuator state (1xn array)
% 
% Outputs:
%   coeff               aerodynamic coefficient as defined by the type
%                       input (1xn array)
% 
% See also: wingAirfoilMapDerivCoeff, wingAirfoilMapSetSim
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2021 Lucas Schreer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% check input variable type
if strcmpi(type,'c_L')
    type = 'c_L';
elseif strcmpi(type,'c_D')
    type = 'c_D';
elseif strcmpi(type,'c_m')
    type = 'c_m';
else
    error('Interpolation only for c_L, c_D and c_m provided')
end

% ndim = scellArrMaxDim(wing.profile_aero.map_cl.dim);
ndim = 5;
% select map
switch type
    case 'c_L'
        [map, airfoil_map.map_cl] = getScellArrAt( airfoil_map.map_cl, segment_type_local );
    case 'c_D'
        [map, airfoil_map.map_cd] = getScellArrAt( airfoil_map.map_cd, segment_type_local );
    case 'c_m'
        [map, airfoil_map.map_cm] = getScellArrAt( airfoil_map.map_cm, segment_type_local );
end

% get grids
[alpha_grid, airfoil_map.alpha]        = getScellArrAt( airfoil_map.alpha, segment_type_local );
[Mach_grid, airfoil_map.Mach]          = getScellArrAt( airfoil_map.Mach, segment_type_local );
[Reynolds_grid, airfoil_map.Reynolds]  = getScellArrAt( airfoil_map.Reynolds, segment_type_local );
[act1_grid, airfoil_map.actuator_1]    = getScellArrAt( airfoil_map.actuator_1, segment_type_local );
[act2_grid, airfoil_map.actuator_2]    = getScellArrAt( airfoil_map.actuator_2, segment_type_local );

% adjust unit
alpha = rad2deg( alpha );

% interpolation of coefficients
% coeff = interpn( alpha_grid, Mach_grid, Reynolds_grid, act1_grid, act2_grid, map, alpha, Ma, Re, act1, act2, 'linear' );
coeff = interpnLinFast( alpha_grid(:)', Mach_grid(:)', Reynolds_grid(:)', act1_grid(:)', act2_grid(:)', map, alpha(:)', Ma(:)', Re(:)', act1(:)', act2(:)', true );

end