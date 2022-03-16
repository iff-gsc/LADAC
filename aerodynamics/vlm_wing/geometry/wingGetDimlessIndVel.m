function v = wingGetDimlessIndVel( V_inf_local, geometry )
% wingGetDimlessIndVel computes the dimensionless induced velocity for a
% unit circulation normalized by the wing span.
%   The implementation of [1], eq. (6) is used. As we assume that no
%   segment of the horseshoe vortices is crossing a control point and the
%   control points do not lie on the bound segment of the horseshoe
%   vortices, we only use the i~=j part and not the i=j part of [1], eq.
%   (6).
%   Moreover, since it is very common, the dimensionless induced velocity
%   is not normalized by the local chord but by the wing span.
% 
% Inputs:
%   V_inf_local         local free-stream velocity for each panel, signs
%                       are defined for how the vector is seen by the plane
%                       (3xn array)
%   geometry            Wing geometry
%                       (struct, see: wingSetGeometry)
%   
% Outputs:
%   v                   dimensionless induced velocity (unit circulation
%                       and normalized by the wing span)
%                       (3 x Nc x Nv array, where Nc is the number of
%                       control points and Nv is the number of horseshoe
%                       vortices)
% 
% Literature:
%   [1] Phillips, W. F., & Snyder, D. O. (2000). Modern adaption of
%       Prandtl's classic lifting-line theory. Jounal of Aircraft, 37(4),
%       662-670.
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% create array of unit vectors of the 
dim = size( V_inf_local, 1 );
v_inf_local = V_inf_local ./ repmat( vecnorm( V_inf_local ), dim, 1 );

% move control points and vortex points in case of sideslip (pretty dumb
% but no so bad solution)
r_cntrl_n = geometry.ctrl_pt.pos;
r_cntrl_n(1,:) = geometry.ctrl_pt.pos(1,:) - v_inf_local(2,:)./vecnorm(v_inf_local,2,1).*geometry.ctrl_pt.pos(2,:);
r_ref = (max(geometry.vortex.pos(1,:))+min(geometry.vortex.pos(1,:)))/2;
diff_r_vortex = geometry.vortex.pos(1,:)-r_ref;
diff_r = diff_r_vortex(1,1:end-1) + diff(diff_r_vortex)/2;
r_cntrl_n(2,:) = geometry.ctrl_pt.pos(2,:) + v_inf_local(2,:)./vecnorm(v_inf_local,2,1).*diff_r;
geometry.ctrl_pt.pos = r_cntrl_n;
v_inf_local_vortex = [v_inf_local,v_inf_local(:,end)];
r_vortex_n = geometry.vortex.pos;
r_vortex_n(1,:) = geometry.vortex.pos(1,:) - v_inf_local_vortex(2,:)./vecnorm(v_inf_local_vortex,2,1).*geometry.vortex.pos(2,:);
r_vortex_n(2,:) = geometry.vortex.pos(2,:) + v_inf_local_vortex(2,:)./vecnorm(v_inf_local_vortex,2,1).*diff_r_vortex;
geometry.vortex.pos = r_vortex_n;
v_inf_local(2,:) = 0;

% get the wing span
span    = sum( wingGetSegmentSpan( geometry.vortex ) );

% Biot-Savart for trailing vortices and bound vortices [1], eq. (6)
AIC = biotSavart( v_inf_local, r_vortex_n, r_cntrl_n );
v = span * AIC;

end