function Z = propMapFitGetZ( prop_fit, RPM, V, output_name )
% PROPMAPFITGETZ get propeller map fit output values at operating points
%   The outputs of the propeller map (thrust, torque or power) can be
%   computed depending on the RPM and the airspeed.
% 
% Syntax:
%   Z = propMapFitGetZ( prop_fit, RPM, V, output_name )
% 
% Inputs:
%   prop_fit                propeller map fit (struct) as defined by the
%                           function propMapFitCreate
%   RPM                     propeller rotational speed (NxM array), in rpm
%   V                       airspeed (NxM array), in m/s
%   output_name             name of the desired output quantity (string):
%                               'thrust' or
%                               'torque' or
%                               'power'
% 
% Outputs:
%   Z                       output value of the propeller map which is
%                           either the thrust or the torque or the power
%                           depending on the function input output_name
%                           (NxM array), in N or Nm or W
% 
% See also:
%   PROPMAPFITCREATE, PROPMAPFITPLOT, PROPMAPFITGETZDERIV
 
% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

[RPM_lim,V_lim] = propMapAvoidExtrap(prop_fit,RPM,V);

switch output_name
    case 'thrust'
        c = prop_fit.coeffs_thrust;
    case {'torque','power'}
        c = prop_fit.coeffs_torque;
end

Z = c.p00 + c.p10.*RPM_lim + c.p01.*V_lim + c.p20.*RPM_lim.^2 ...
    + c.p11.*RPM_lim.*V_lim + c.p02.*V_lim.^2 + c.p30.*RPM_lim.^3 ...
    + c.p21.*RPM_lim.^2.*V_lim + c.p12.*RPM_lim.*V_lim.^2 + c.p03.*V_lim.^3 ...
    + c.p40.*RPM_lim.^4 + c.p31.*RPM_lim.^3.*V_lim + c.p22.*RPM_lim.^2.*V_lim.^2 ...
    + c.p13.*RPM_lim.*V_lim.^3 + c.p04.*V_lim.^4 + c.p50.*RPM_lim.^5 ...
    + c.p41.*RPM_lim.^4.*V_lim + c.p32.*RPM_lim.^3.*V_lim.^2 ...
    + c.p23.*RPM_lim.^2.*V_lim.^3 + c.p14.*RPM_lim.*V_lim.^4 + c.p05.*V_lim.^5;

Z_dRPM = propMapFitGetZDeriv( prop_fit, RPM_lim, V_lim, output_name, 'RPM', true );
Z_dV = propMapFitGetZDeriv( prop_fit, RPM_lim, V_lim, output_name, 'V', true );

% apply linear extrapolation
Z = Z + Z_dRPM.*(RPM-RPM_lim) + Z_dV.*(V-V_lim);

switch output_name
    case 'power'
        Z = Z.*(RPM_lim*2*pi/60);
    otherwise
        return
end

end