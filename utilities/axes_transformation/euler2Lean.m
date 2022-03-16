function [phi,delta] = euler2Lean(roll_cmd,pitch_cmd)

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

ry = roll_cmd;
px = -pitch_cmd;

delta = atan2(ry,px);

delta_abs = abs(delta);
up = max( -1, min( 1, tan(pi/2-delta_abs) ) );
right = max( -1, min( 1, tan(delta_abs) ) );
dist = sqrt( up^2 + right^2 );

phi = sqrt(px^2+ry^2) / dist;

end