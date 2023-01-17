function angle_rad = deg2rad( angle_deg ) %#codegen
% rad2deg converts an angle in degree into rad.

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

    angle_rad = angle_deg * pi/180;

end