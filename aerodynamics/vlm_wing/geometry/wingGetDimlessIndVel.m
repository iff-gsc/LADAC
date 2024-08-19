function [v_b,v_t] = wingGetDimlessIndVel( V_inf_local, geometry, wake, Ma, is_unsteady )
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

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Ma = 0.7417*0;
% Ma = 0.6928*0;
% Ma = 0.6128*0;
% Ma(:) = cosd(22)*0.8;
beta_Ma = 1./sqrtReal(1-Ma.^2);
cntrl_pt = geometry.ctrl_pt.pos + V_inf_local .* 0.5 .* geometry.ctrl_pt.c .* ( beta_Ma - 1 );
geometry.vortex.pos(:,:,2:end) = geometry.vortex.pos(:,:,2:end) + (geometry.vortex.pos(:,:,2:end)-geometry.vortex.pos(:,:,1) ) .* ( [beta_Ma(1),beta_Ma(1:end-1)+0.5*diff(beta_Ma),beta_Ma(end)] - 1 );

% create array of unit vectors of the 
dim = size( V_inf_local, 1 );
n_vxb = size( V_inf_local, 3 );
v_inf_local = V_inf_local ./ repmat( vecnorm( V_inf_local ), dim, 1 );

% move control points and vortex points in case of sideslip (pretty dumb
% but no so bad solution)
r_cntrl_n = cntrl_pt;
r_cntrl_n(1,:) = cntrl_pt(1,:) - v_inf_local(2,:)./vecnorm(v_inf_local(:,:),2,1).*cntrl_pt(2,:);
r_ref = (max(geometry.vortex.pos(1,:))+min(geometry.vortex.pos(1,:)))/2;

diff_r_vortex = geometry.vortex.pos(1,:,:)-r_ref;
    
diff_r = diff_r_vortex(1,1:end-1,1:end-1) + diff(diff(diff_r_vortex),1,3)/2;
r_cntrl_n(2,:) = cntrl_pt(2,:) + v_inf_local(2,:)./vecnorm(v_inf_local(:,:),2,1).*diff_r(1,:);
cntrl_pt = r_cntrl_n;
v_inf_local_vortex = cat( 3, [v_inf_local,v_inf_local(:,end,:)], [v_inf_local(:,:,end),v_inf_local(:,end,end)] );
r_vortex_n = geometry.vortex.pos;
r_vortex_n(1,:) = geometry.vortex.pos(1,:) - v_inf_local_vortex(2,:)./vecnorm(v_inf_local_vortex(:,:),2,1).*geometry.vortex.pos(2,:);
r_vortex_n(2,:) = geometry.vortex.pos(2,:) + v_inf_local_vortex(2,:)./vecnorm(v_inf_local_vortex(:,:),2,1).*diff_r_vortex(1,:);
geometry.vortex.pos = r_vortex_n;
v_inf_local(2,:) = 0;

% get the wing span
span    = sum( wingGetSegmentSpan( geometry.line_25 ) );

if n_vxb == 0 % 1
    % Biot-Savart for trailing vortices and bound vortices [1], eq. (6)
%     AIC = biotSavart( v_inf_local, r_vortex_n, r_cntrl_n );
%     v = span * AIC;
else

    n_cy = size(cntrl_pt,2);
    n_cx = size(cntrl_pt,3);
    n_vy = size(geometry.vortex.pos,2)-1;
    n_vxt = length(wake.pos_x)-1;
    n_vx = n_vxb + n_vxt;

    n_vb = n_vxb * n_vy;
    n_vt = n_vxt * n_vy;
    n_c = n_cx * n_cy;

    AIC_b = zeros( 3, n_c, n_vb );
    AIC_t = zeros( 3, n_c, n_vt );

    for k = 1:n_cx
        for l = 1:n_cy
            for j = 1:n_vy
                for i = 1:n_vxb
                
                    uvw_ij = vlmVoring( ...
                                geometry.vortex.pos(:,j,i), ...
                                geometry.vortex.pos(:,j+1,i), ...
                                geometry.vortex.pos(:,j+1,i+1), ...
                                geometry.vortex.pos(:,j,i+1), ...
                                cntrl_pt(:,l,k), ...
                                1 );
                    idx1 = (k-1)*n_cy + l;
                    idx2b = (i-1)*n_vy + j;
                    AIC_b(:,idx1,idx2b) = uvw_ij;
                end
                for ii = 1:n_vxt
                    uvw_iij = vlmVoring( ...
                                geometry.vortex.pos(:,j,end) + [-wake.pos_x(ii);0;0], ...
                                geometry.vortex.pos(:,j+1,end) + [-wake.pos_x(ii);0;0], ...
                                geometry.vortex.pos(:,j+1,end) + [-wake.pos_x(ii+1);0;0], ...
                                geometry.vortex.pos(:,j,end) + [-wake.pos_x(ii+1);0;0], ...
                                cntrl_pt(:,l,k), ...
                                1 );
                    idx1 = (k-1)*n_cy + l;
                    idx2t = (ii-1)*n_vy + j;
                    AIC_t(:,idx1,idx2t) = uvw_iij;
                end
            end
        end
    end

    v_b = span * AIC_b;
    v_t = span * AIC_t;
end

end