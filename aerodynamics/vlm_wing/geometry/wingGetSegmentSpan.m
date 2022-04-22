function db = wingGetSegmentSpan( vortex )


% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

spanwise_diff = diff( vortex.pos(2:3,:), 1, 2 );
db = vecnorm( spanwise_diff, 2, 1 );

end