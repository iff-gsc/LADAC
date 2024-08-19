function error = errorAngle(angle_ref,angle) %#codegen
% errorAngle computes the error between two angles wrapped to a rage of 0
% to 2*pi.
% 
% Inputs
%   angle_ref   reference angle between 0 and 2*pi, in rad
%   angle       measured angle between 0 and 2*pi, in rad
% 
% Outputs
%   error       difference between the angles (angle_ref - angle) in a
%               range of -pi to pi, in rad
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% compute the unwrapped error
error = angle_ref - angle;

% there are to cases
if error > pi
    error = error - 2*pi;
elseif error < -pi
    error = error + 2*pi;
end

end

