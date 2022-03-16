function assignment_vector = fuselageAssignToStructure( geometry, structure )
%fuselageAssignToStructure find corresponding points of fuselage geometry
%and a structure.
%   The fuselage geometry struct is used for aerodynamics computations
%   while a structure struct is used for structural dynamics computations.
%   In order to connect these and to consider aeroelasticity, each point of
%   the structure must be assigned to a point of the fuselage geometry.
% 
% Inputs:
%   geometry            fuselage geometry struct (see fuselageGeometryInit)
%   structure           structure struct (see structureCreateFromNastran)
% 
% Outputs:
%   assignment_vector   assignment of aerodynamic points to the structure
%                       points (1xM, array of integers where M is the 
%                       number of structure points); assignment_vector(i)
%                       is the number/index of the aerodynamic point that 
%                       is assigned to the i-th structure point;
%                       if assignment(i) == 0, no aerodynamic point was
%                       assigned to the i-th structure point;
%                       note that 0 <= assignment(i) <= N, where N is the
%                       number of aerodynamic points
% 
% See also:
%   fuselageSetAeroelasticity, fuselageInit, fuselageCreate
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

xyz_s = structure.xyz;
xyz_f = geometry.cntrl_pos;

assignment_vector = zeros(1,size(xyz_s,2));

for i=1:size(xyz_f,2)
    convex_solid = fuselageSegment2convexSolid(geometry,i);
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

function convexSolid = fuselageSegment2convexSolid(fuseGeometry,idx)

convexSolid = zeros(4,6); % six planes (hexaeder)

mean_diameter = sqrt(mean(fuseGeometry.width(idx:idx+1).^2));
mean_length = mean_diameter/2 * sqrt(2)/2;
cntrl_pos = fuseGeometry.cntrl_pos(:,idx);
border_front = fuseGeometry.border_pos(:,idx);
border_back = fuseGeometry.border_pos(:,idx+1);

% upper plane
u_n_1 = [0;0;-1];
d_1 = - dot( u_n_1, cntrl_pos + [0;0;-mean_length] );

% lower plane
u_n_2 = -u_n_1;
d_2 = - dot( u_n_2, cntrl_pos + [0;0;mean_length] );

% right plane
u_n_3 = [0;1;0];
% u_n_3 = xyz_vortex/norm(xyz_vortex,2);
d_3 = - dot( u_n_3, cntrl_pos + [0;mean_length;0] );

% left plane
u_n_4 = -u_n_3;
d_4 = - dot( u_n_4, cntrl_pos + [0;-mean_length;0] );

% front plane
u_n_5 = [1;0;0];
d_5 = - dot( u_n_5, border_front );

% back plane
u_n_6 = -u_n_5;
d_6 = -dot( u_n_6, border_back );

convexSolid(1:3,:) = [u_n_1,u_n_2,u_n_3,u_n_4,u_n_5,u_n_6];
convexSolid(4,:) = [d_1,d_2,d_3,d_4,d_5,d_6];
    
end