function uvw_mat = vlmVoringsLlt( pv, pc, Gamma, dist )
% vlmVoringsLlt computes the induced velocity vectors at given control
% points with lifting line theory
%   The wing is assumed to move in x-direction, so the horseshoe vortices
%   are parallel to the x-axis.
% 
% Syntax:
%   uvw_mat = vlmVoringsLlt( pv, pc, Gamma, dist )
% 
% Inputs:
%   pv          Corner position vectors of horseshoe vortices, where the
%               wing is assumed to move in x-direction (3xN+1 matrix for N
%               horseshoe vortices), in m
%   pc          Position vectors of control points, where the induces
%               velocities are to be computed (3xM matrix for M control
%               points), in m
%   Gamma       Circulation of all horseshoe vortices (1xN matrix), in
%               m^2/s
%   dist        Length of the horseshoe vortices parallel to x-axis
%               (scalar), in m (the value should be much greater than the
%               wing span)
% 
% Outputs:
%   uvw_mat     Induced velocity vectors at positions pc (3xMxN matrix), in
%               m/s
% 
% See also:
%   vlmVoring, wingGetDownwash

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

num_vor = size(pv,2)-1;
num_cntrl = size(pc,2);
uvw_mat = zeros( 3, num_cntrl, num_vor );
for j = 1:num_vor
    vortex_p1 = pv(:,j,1);
    vortex_p2 = pv(:,j+1,1);
    vortex_p3 = pv(:,j+1,1) + [-dist;0;0];
    vortex_p4 = pv(:,j,1) + [-dist;0;0];
    for i = 1:num_cntrl
        cntrl_pt = pc(:,i);
        uvw_ij = vlmVoring( vortex_p1, vortex_p2, vortex_p3, vortex_p4, ...
                    cntrl_pt, Gamma );
        uvw_mat(:,i,j) = uvw_ij;
    end
end

end