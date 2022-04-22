function lift2bm = wingGetBendingMoment( wing_geometry, eta )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

num_eta = length(eta);
num_cntrl_pt = length(wing_geometry.ctrl_pt.pos(1,:));

lift2bm = zeros( num_eta, num_cntrl_pt );

% bending moment at each control point...
% ... at the left side
cntrl_span = -vecnorm(wing_geometry.ctrl_pt.pos(2:3,:),2,1);
cntrl_span([1,diff(cntrl_span)]<0) = -cntrl_span([1,diff(cntrl_span)]<0);

span = 2 * norm(wing_geometry.vortex.pos(2:3,end),2);

% R_Ab_i = wingGetLocalForce( wing );
% lift = vecnorm(R_Ab_i(2:3,:),2,1);

for i = 1:num_eta
    if eta(i) < 0
        is_used = cntrl_span < eta(i) * span/2;
    else
        is_used = cntrl_span > eta(i) * span/2;
    end
    lift2bm(i,is_used) = abs(cntrl_span(is_used)) - abs(eta(i)*span/2);    
end

end
