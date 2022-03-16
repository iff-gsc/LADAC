function [eta_eff] = simpleWingGetEffFlapAng(wing, eta) %#codegen
% simpleWingGetEffFlapAng calculates the effective flap angle
%   Background information on the effective flap angle can be found in
%   ([1], page 441, figure 12.9). This function calculates the effective
%   flap angle based on a scaled sin() function. If a new model is created,
%   a suitable scaling factor must be found! There are two ways to find a
%   scaling factor:
%     - Try and Error:
%         Try different scaling factors and watch the results with Example 2.
%     - Angle / Offset data point:
%         Define an offset for a specific angle and calculate the
%         needed scaling factor as shown in Example 1.
%   If the effective flap angle should not be used at all, do not define
%   the eff_scaling field in the wing.flap struct.
% 
% Syntax:
%   [eta_eff] = simpleWingGetEffFlapAng( wing, eta )
%
% Inputs:
%   wing            simple wing (struct) as defined by the function
%                   simpleWingLoadParams
%   eta             flap deflection angle (scalar or vector), in rad
%
% Outputs:
%   eta_eff         effective flap deflection angle (same dimension as
%                   input), in rad
%
% Examples:
% Define parameters:
%   % define simple wing characteristic (default)
%   wing = simpleWingLoadParams('params_aero_simple_wing_default')
%   % define the desired scaling factor
%   wing.flap.eff_scaling = <desired scaling factor>;
%
% Example 1 (Calculate scaling factor)
%   If you want to calculate a scaling factor that meets a defined
%   offset at a specific angle, proceed as follows:
%       - You need the "Symbolic Math Toolbox" for the following calculations!
%       - Only negative offsets are possible.
%
%   angle = <specific angle, in deg>;
%   offset = <offset at specific angle, in deg>;
%   syms x;
%   y = (angle + offset) / angle;
%   scaling = abs(double(solve(sinc(deg2rad(angle)*x) == y))) * pi;
%
% Example 2 (Visualize effective flap angle)
%     eta_deg = linspace(<actuator min>, <actuator max>);
%     eta_eff_deg = rad2deg(simpleWingGetEffFlapAng(wing, deg2rad(eta_deg)));
%     figure()
%     hold on;
%     grid on;
%     axis equal;
%     plot(eta_deg, eta_deg, 'k--');
%     plot(eta_deg, eta_eff_deg)
% 
% Literature:
%   [1] Schlichting, H. & Truckenbrodt, E. (2001): Aerodynamik des
%       Flugzeuges - Teil 2, Springer.
% 
% See also: simpleWingGetCl, simpleWingGetCd

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2021 Jonas Withelm
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Execute warning() as extrinsic function
coder.extrinsic('warning');

% Read scaling factor out of wing struct
scaling = wing.flap.eff_scaling;

% Check if input angle is too big
if max(abs(eta)) > (deg2rad(90) / scaling)
    warn_str = sprintf(['In simpleWingGetEffFlapAng():\n',...
             'Input angle too big, simpleWingGetEffFlapAng() delivers ',...
             'false results! Please check wing.flap.eff_scaling factor ',...
             'in combination with actuator boundaries.']);
    warning(warn_str);
end

% Calculate effective flap angle
eta_eff = sin( eta*scaling ) * ( 1/scaling );

end
