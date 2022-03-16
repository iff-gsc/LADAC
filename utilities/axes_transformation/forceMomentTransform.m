%forceMomentTransform    transforms a force and moment into a force and 
% moment in a different frame
% This function transforms a force applied at the origin of an arbitrary 
% frame x and a moment into an arbitrary frame y using a direction cosine 
% matrix. Moreover, due to the lever arm of the force about the origin of 
% frame y, the resulting moment in frame y is computed.
%
% Inputs:
%   R_x         the force vector applied at the origin of frame x 
%               represented in frame x, in N
%   Q_x         the moment vector represented in frame x, in N
%   r_x_y       the position vector of the origin of frame x represented in
%               frame y, in m 
%   M_yx        the direction cosine matrix for the rotation from frame x
%               to frame y, in 1
%
% Outputs:
%   R_y         the force vector represented in frame y, in N
%   Q_y         the resulting moment vector represented in frame y, in Nm
%
% See also: euler2Dcm
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function [ R_y, Q_y ] = forceMomentTransform( R_x, Q_x, r_x_y, M_yx )

% determine number of input vectors
num_vectors = length( r_x_y(1,:) );
R_y = zeros( 3, num_vectors );
Q_y = zeros( 3, num_vectors );

if length(size(M_yx)) == 2
    M_yx_3d = repmat( M_yx, 1, 1, num_vectors );
else
    M_yx_3d = M_yx;
end

for i = 1:num_vectors
    % transformation of the force from frame x to frame y
    R_y(:,i) = M_yx_3d(:,:,i) * R_x(:,i);
    % computation of the resulting moment represented in frame y
    Q_y(:,i) = M_yx_3d(:,:,i) * Q_x(:,i) + cross( r_x_y(:,i), R_y(:,i) );
end
end