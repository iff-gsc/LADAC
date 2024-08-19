function Z_du = propMapFitGetZDeriv( prop_fit, RPM, V, output_name, ...
    deriv_name, is_extrap )
% PROPMAPFITGETZDERIV get propeller map fit output partial derivative
%   The outputs of the propeller map (thrust, torque or power) can be
%   derived w.r.t. the inputs RPM and V at the specified operating point.
% 
% Syntax:
%   Z = propMapFitGetZDeriv( prop_fit, RPM, V, output_name )
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
%   deriv_name              name of the variable (string); partial
%                           derivative will be computed w.r.t. this
%                           variable:
%                               'RPM' or
%                               'V'
%   is_extrap               (optional) if the operating point is outside of
%                           the map data, should be extrapolated (true) or
%                           clipped (false) (bool); default is false
% 
% Outputs:
%   Z_du                    partial derivative of the specified input 
%                           output_name w.r.t. the specified input 
%                           deriv_name (NxM array), in N/rpm or Nm/rpm or
%                           W/rpm or N/(m/s) or Nm/(m/s) or W/(m/s)
% 
% See also:
%   PROPMAPFITCREATE, PROPMAPFITPLOT, PROPMAPFITGETZ
 
% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if nargin < 6
    is_extrap = false;
end

if is_extrap
    RPM_lim = RPM;
else
    [RPM_lim,V] = propMapAvoidExtrap(prop_fit,RPM,V);
end

switch output_name
    case 'thrust'
        c = prop_fit.coeffs_thrust;
    case {'torque','power'}
        c = prop_fit.coeffs_torque;
end

switch deriv_name
    case 'RPM'
        Z_du = c.p10 + 2*c.p20.*RPM_lim + c.p11.*V + 3*c.p30.*RPM_lim.^2 ...
            + 2*c.p21.*RPM_lim.*V + c.p12.*V.^2 + 4*c.p40.*RPM_lim.^3 ...
            + 3*c.p31.*RPM_lim.^2.*V + 2*c.p22.*RPM_lim.*V.^2 + c.p13.*V.^3 ...
            + 5*c.p50.*RPM_lim.^4 + 4*c.p41.*RPM_lim.^3.*V ...
            + 3*c.p32.*RPM_lim.^2.*V.^2 + 2*c.p23.*RPM_lim.*V.^3 + c.p14.*V.^4;
    case 'V'
        Z_du = c.p01 + c.p11.*RPM_lim + 2*c.p02.*V + c.p21.*RPM_lim.^2 ...
            + 2*c.p12.*RPM_lim.*V + 3*c.p03.*V.^2 + c.p31.*RPM_lim.^3 ...
            + 2*c.p22.*RPM_lim.^2.*V + 3*c.p13.*RPM_lim.*V.^2 + 4*c.p04.*V.^3 ...
            + c.p41.*RPM_lim.^4 + 2*c.p32.*RPM_lim.^3.*V ...
            + 3*c.p23.*RPM_lim.^2.*V.^2 + 4*c.p14.*RPM_lim.*V.^3 + 5*c.p05.*V.^4;
end

switch output_name
    case 'power'
        switch deriv_name
            case 'RPM'
                Z_du = Z_du.*(RPM*2*pi/60) ...
                    + propMapFitGetZ(prop_fit,RPM_lim,V,'torque') .* (2*pi/60);
            case 'V'
                Z_du = Z_du.*(RPM*2*pi/60);
        end
    otherwise
        return
end
                
end
