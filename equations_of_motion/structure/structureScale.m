function structure = structureScale( structure, scale_geo, scale_rho )
% structureScale scales a structure dynamics model for different size and
% air density
%   The scaling is applied such that the "Froude number" and the "Relative
%   Density" are matched [1].
% 
% Syntax:
%   structure = structureScale( structure, scale_geo, scale_rho )
% 
% Inputs:
%   structure           structure struct as specified by
%                       structureCreateFromNastran
%   scale_geo           Geometric scaling factor (scalar), e.g. 0.5 if the
%                       geometry should be reduced to 50%
%   scale_rho           Air density scaling factor (scalar), e.g. 2 if the
%                       air density should be doubled
% 
% Outputs:
%   structure           structure struct as specified by
%                       structureCreateFromNastran
% 
% Literature:
%   [1] Bo, T., Zhigang, W., & Chao, Y. (2015). Aeroelastic scaling laws
%       for gust load alleviation.
% 
% See also:
%   structureCreateFromNastran

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

scale_mass = scale_geo^3 * scale_rho;
scale_stiff_lin = scale_mass / scale_geo;
scale_stiff_rot = scale_geo * scale_mass;
scale_stiff_coupl = scale_mass;

structure.xyz = structure.xyz * scale_geo;
structure.M = structure.M * scale_mass;
num_nodes = size(structure.xyz,2);
I_node = structureGetNodeInertia( structure, 1:num_nodes );
I_node = I_node * scale_geo^2;
structure = structureSetNodeInertia( structure, I_node, 1:num_nodes );
cg_node = structureGetNodeCg( structure, 1:num_nodes );
cg_node = cg_node * scale_geo;
structure = structureSetNodeCg( structure, 1:num_nodes, cg_node );
for i = 1:num_nodes
    idx_row = (i-1)*6+1:i*6;
    for j = 1:num_nodes
        idx_col = (j-1)*6+1:j*6;
        K_sub = structure.K(idx_row,idx_col);
        K_sub(1:3,1:3) = K_sub(1:3,1:3) * scale_stiff_lin;
        K_sub(4:6,4:6) = K_sub(4:6,4:6) * scale_stiff_rot;
        K_sub(1:3,4:6) = K_sub(1:3,4:6) * scale_stiff_coupl;
        K_sub(4:6,1:3) = K_sub(4:6,1:3) * scale_stiff_coupl;
        structure.K(idx_row,idx_col) = K_sub;
    end
end

end