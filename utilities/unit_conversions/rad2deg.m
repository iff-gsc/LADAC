function angle_deg = rad2deg( angle_rad ) %#codegen
% rad2deg converts an angle in rad into degree.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    angle_deg = angle_rad * 180/pi;

end