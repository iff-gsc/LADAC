function n_g = leanAngles2leanVector( leanAngles )
% leanAngles2leanVector converts the reduced attitude represented as angles
%   into the reduced attitude represented as unit vector.
%   For more information about the reduced attitude vector see [1], section
%   II.B.
% 
% Inputs:
%   phi                 tilt angle (see dcm2Lean), rad
%   delta               direction of the tilt angle (see dcm2Lean), rad
% 
% Outputs:
%   n_g                 unit vector that points into the -z_b direction
%                       represented in g frame (NED)
% 
% Literature:
%   [1] https://arxiv.org/pdf/2002.07837.pdf
% 
% See also:
%   quatReduced, euler2Dcm, quat2Dcm
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

phi = leanAngles(1);
delta = leanAngles(2);

n_z_g = -cos( phi );

n_xy_g = sin( phi );

n_x_g = n_xy_g * cos( delta );
n_y_g = n_xy_g * sin( delta );

n_g = [ n_x_g; n_y_g; n_z_g ];

end