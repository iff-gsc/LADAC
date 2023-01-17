function angle_0_2pi = wrapAngle(angle) %#codegen
% wrapAngle wraps angles to the range 0 to 2*pi.
% 
% Inputs
%   angle           angle, in rad
% % 
% Outputs
%   angle_0_2pi     angle in the rage of 0 to 2*pi, in rad
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% custom modulo (for C++ code generation): mod_heading =
% mod(abs(heading),2*pi)
x = abs( angle );
y = 2*pi;
n = floor(x./y);
m = x - n.*y;
mod_heading = m;

% there are two cases
if angle >= 0
    angle_0_2pi = mod_heading;
else
    angle_0_2pi = 2*pi - mod_heading;
end

end