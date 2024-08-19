function p = structureGetLoadVectorFromNodeLoad( force_matrix, moment_matrix )
% structureGetLoadVectorFromNodeLoad converts the local node forces and
% moments to the complete load vector.
% 
% Syntax:
%   p = structureGetLoadVectorFromNodeLoad( force_matrix, moment_matrix )
% 
% Inputs:
%   force_matrix        concentrated node force vectors in structure frame
%                       (3xN array with N nodes)
%   moment_matrix       concentrated node moment vectors in structure frame
%                       (3xN array with N nodes)
% 
% Outputs:
%   p                   load vector, where the first 6 elements are the
%                       loads at the first node (3x force, 3x moment), the
%                       next 6 elements are the loads at the second node
%                       and so on in structure frame ((6*N)x1 array with N
%                       nodes)
% 
% See also:
%   structureCreateFromNastran, structureGetNodeMass

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

p = reshape( [force_matrix;moment_matrix], [], 1 );

end