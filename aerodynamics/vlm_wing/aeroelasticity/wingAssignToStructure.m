function assignment_vector = wingAssignToStructure( wingGeometry, structure )

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

wing_origin = wingGeometry.origin;
xyz_s = structure.xyz;
xyz_w = wingGeometry.ctrl_pt.pos + wing_origin;

assignment_vector = zeros(1,size(xyz_s,2));

for i=1:size(xyz_w,2)
    convex_solid = wingSegment2convexSolid(wingGeometry,wing_origin,i);
    for j=1:size(xyz_s,2)
        is_inside = convexSolidContainsPoint(xyz_s(:,j),convex_solid);
        if is_inside
            assignment_vector(j) = i;
        end
    end

end

end

% https://stackoverflow.com/questions/34379859/check-if-a-3d-point-lies-inside-a-3d-platonic-solid
function side = side(point,plane)
    v = point(1)*plane(1) + point(2)*plane(2) + point(3)*plane(3) + plane(4);
    if v > eps(v)
        side = 1;
    elseif v < -eps(v)
        side = -1;
    else
        side = 0;
    end
end

function contains = convexSolidContainsPoint(point,convexSolid)
    contains = true;
    num_planes = size(convexSolid,2);
    for k=1:num_planes
        side_ = side(point,convexSolid(:,k));
        if side_ > 0
            contains = false;
            break;
        end
    end
end

function convexSolid = wingSegment2convexSolid(wingGeometry,origin,idx)
wing_rel_thickness = 0.1;
convexSolid = zeros(4,6); % six planes (hexaeder)

chord_left = wingGeometry.line_25.c(idx);
chord_right = wingGeometry.line_25.c(idx+1);
chord = 0.5*(chord_left+chord_right);

thickness = chord * wing_rel_thickness;

in = wingGeometry.ctrl_pt.local_incidence(idx);
s_in = sin(in);
c_in = cos(in);
xyz_lead = wingGeometry.line_25.pos(:,idx:idx+1) + ...
    origin + ...
    [c_in,0,s_in;0,1,0;-s_in,0,c_in] * [chord_left,chord_right;0,0;0,0]/4;
xyz_trail = wingGeometry.line_25.pos(:,idx:idx+1) + ...
    origin - ...
    [c_in,0,s_in;0,1,0;-s_in,0,c_in] * [chord_left,chord_right;0,0;0,0]*3/4;

% bound_vortex_vector = wingGetSpatialVectorAlongBoundSegment(wingGeometry.vortex,idx:idx+1);
point_vortex_left = wingGeometry.line_25.pos(:,idx) + origin;
point_vortex_right = wingGeometry.line_25.pos(:,idx+1) + origin;

% upper plane
u_n_1 = wingGetNormalVectorFromGeometry(wingGeometry,idx);
d_1 = - dot( u_n_1, point_vortex_left + u_n_1*thickness/2 );

% lower plane
u_n_2 = -u_n_1;
d_2 = - dot( u_n_2, point_vortex_left + u_n_2*thickness/2 );

% right plane
chord_vector = xyz_lead(:,2) - xyz_trail(:,2);
u_n_3 = cross( chord_vector/norm(chord_vector), u_n_1 );
% u_n_3 = xyz_vortex/norm(xyz_vortex,2);
d_3 = - dot( u_n_3, point_vortex_right );

% left plane
u_n_4 = -u_n_3;
d_4 = - dot( u_n_4, point_vortex_left );

% front plane
u_n_5 = cross(diff(xyz_lead,1,2),u_n_2);
d_5 = - dot( u_n_5, xyz_lead(:,1) );

% back plane
u_n_6 = cross(diff(xyz_trail,1,2),u_n_1);
d_6 = -dot( u_n_6, xyz_trail(:,1) );

convexSolid(1:3,:) = [u_n_1,u_n_2,u_n_3,u_n_4,u_n_5,u_n_6];
convexSolid(4,:) = [d_1,d_2,d_3,d_4,d_5,d_6];
    
end