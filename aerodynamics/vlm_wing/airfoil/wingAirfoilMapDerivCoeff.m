function [coeff_dArg,airfoil_map] = wingAirfoilMapDerivCoeff( airfoil_map, segment_type_local, type, arg, alpha, Ma, Re, act1, act2  ) %#codegen
% wingAirfoilMapDerivCoeff gets the derivative of an aerodynamic coefficient
% with respect to a specified variable from a profile map.
%   Since there are big problems with code generation if multiple
%   aerodynamic maps are used in Simulink, the profile_map is an Scell
%   array (see cell2scellArr, mat_file_format.md).
%   The derivatives are computed by numeric linearization at the specified
%   location(s) in the map.
% 
% Inputs:
%   airfoil_map         Scell array containing the profile map as explained
%                       in mat_file_format.md.
%   segment_type_local  Index that specifies the profile (needed for
%                       support of multiple profiles per wing).
%   type                type of coefficient used: 'c_L', 'c_D' or 'c_m'
%   arg                 specify to which variable you want to get the
%                       derivative: 'alpha', 'Ma', 'Re', 'act1', 'act2'
%   alpha               angle of attack (1xn array)
%   delta               flap deflection or actuator state (1xn array)
%   Ma                  Mach number (1xn array)
%   Re                  Reynolds number (1xn array)
% 
% Outputs:
%   coeff_dArg      	derivative of specified aerodynamic coefficient
%                       (type) with respect to specified variable (arg)
%                       (1xn array)
% 
% See also: wingAirfoilMapInterpCoeff, wingAirfoilMapSetSim
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% init pertubation magnitude
delta_alpha = 0;
delta_act1 = 0;
delta_act2 = 0;
delta_Ma = 0;
delta_Re = 0;

% overwrite desired pertubation magnitude
switch arg
    case 'alpha'
        delta = 0.01;
        delta_alpha = delta;
    case 'Ma'
        delta = 0.01;
        delta_Ma = delta;
    case 'Re'
        delta = 100;
        delta_Re = delta;
    case 'act1'
        delta = 0.01;
        delta_act1 = delta;
    case 'act2'
        delta = 0.01;
        delta_act2 = delta;
end

% positive and negative pertubated coefficient
coeff_up = wingInterpProfileCoeff( airfoil_map, segment_type_local, type, ...
    alpha+delta_alpha, Ma+delta_Ma, Re+delta_Re, act1+delta_act1, act2+delta_act2 );
coeff_down = wingInterpProfileCoeff( airfoil_map, segment_type_local, type, ...
    alpha-delta_alpha, Ma-delta_Ma, Re-delta_Re, act1-delta_act1, act2-delta_act2 );

% central difference
coeff_dArg = ( coeff_up - coeff_down ) / ( 2*delta );

end

