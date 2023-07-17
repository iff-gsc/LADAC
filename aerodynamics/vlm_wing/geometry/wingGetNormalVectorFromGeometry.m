function u_n_b = wingGetNormalVectorFromGeometry( geometry, varargin )

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% u_n_b = cross( geometry.vortex.pos(:,1:end-1,1:end-1) - geometry.ctrl_pt.pos, diff(geometry.vortex.pos(:,:,1:end-1),1,2) );
% u_n_b = cross( diff(geometry.vortex.pos(:,1:end-1,:),1,3)+diff(geometry.vortex.pos(:,2:end,:),1,3), diff(geometry.vortex.pos(:,:,1:end-1),1,2)+diff(geometry.vortex.pos(:,:,2:end),1,2) );
% 
% u_n_b = u_n_b ./ vecnorm(u_n_b,2,1);
% 
% return;

% spatial vector along bound segment
dl = wingGetSpatialVectorAlongBoundSegment(geometry.line_25);

if ~isempty(varargin)
    idx = varargin{1};
    incidence = geometry.ctrl_pt.local_incidence(idx);
    ny = atan2(dl(3,idx),dl(2,idx));
    pos_vortex = geometry.line_25.pos(:,[idx,idx(end)+1]);
else
    incidence = geometry.ctrl_pt.local_incidence;
    % local dihedral (positive for left wing side)
    ny = atan2(dl(3,:),dl(2,:));
    pos_vortex = geometry.line_25.pos;
end
% The computation of the normal vector is not that easy...
% Rotate two unit vectors in the local wing plane from local wing frame to
% body frame (assume that the local wing plane is unswept!):
% e1 = [1,0,0;0,cn,-sn;0,sn,cn]*[ci,0,si;0,1,0;-si,0,ci]*[1;0;0]
% e2 = [1,0,0;0,cn,-sn;0,sn,cn]*[ci,0,si;0,1,0;-si,0,ci]*[0;1;0]
% with ci = cos(incidence), si = sin(indicence, cn = cos(ny), sn = sin(ny)
% but this is more efficient:
% e1 = [cos(incidence);sin(ny).*sin(incidence);-cos(ny).*sin(incidence)].
% Now, compute the normal vector in body frame: u_n = cross(e2,e1).
% However, this normal vector does not consider the wing sweep!
% The wing sweep in this program is defined such that only the x-coordinate
% is shifted (body to local: ny -> incidence -> sweep).
% How does the normal vector change if the x shift due to sweep is
% considered?
% cross([e2(1)+Delta_x;e2(2);e2(3)],e1(1)) = 
% cross(e2,e1) + [0;Delta_x*(-e1(3));Delta_x*(e1(2))]
% with Delta_x = -sin(sweep).
% Last but not least, the normal vector must be normalized to a unit vector

e1 = [cos(incidence);sin(ny).*sin(incidence);-cos(ny).*sin(incidence)];
% e2 = [zeros(size(ny));cos(ny);sin(ny)];
% cross_e2_e1 = cross(e2,e1);
% -> cross product in above line seems to slow down simulation!
cross_e2_e1 = [-sin(incidence);sin(ny).*cos(incidence);-cos(incidence).*cos(ny)];
sweep = atan(-diff(pos_vortex(1,:))./vecnorm(diff(pos_vortex(2:3,:),1,2)));
Delta_x = -sin(sweep);
% sweep compensation
cross_e2_e1(2,:) = cross_e2_e1(2,:) + Delta_x .* (-e1(3,:));
cross_e2_e1(3,:) = cross_e2_e1(3,:) + Delta_x .* e1(2,:);
u_n_b = cross_e2_e1./repmat(vecnorm(cross_e2_e1),3,1);

end
