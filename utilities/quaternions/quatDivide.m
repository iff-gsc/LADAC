function q_out = quatDivide(q, qr)
% quatDivide divide two quaternions
%   q_out = quatDivide( q, qr ) computes the division of two 
%   quaternions in the form of [w; x; y; z]. With the scalar part w and
%   vector parts x, y and z.
% 
% Syntax:
%   q_out = quatDivide( q, qr )
% 
% Inputs:
%   q               quaternion (4x1 array), dimensionless
%   qr              quaternion (4x1 array), dimensionless
% 
% Outputs:
%   q_out           result of division (4x1 array), dimensionless
%                    
% Example: 
%   q  = [0.7071; 0.7071; 0; 0];
%   qr = [0.7071; -0.7071; 0; 0];
%   q_out = quatDivide(q, qr)
% 
% See also:
%   quatMultiply, quatInv, quatNorm
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% Invert quaternion q

% Conjugate
qr_conj = [qr(1); -qr(2); -qr(3); -qr(4)];

% Normalize
qr = qr_conj ./ (max( eps, qr_conj'*qr_conj));

% Perform multiplication

% Vector part of quaternion product
vec_part = [qr(1)*q(2); qr(1)*q(3); qr(1)*q(4)] + ...
           [q(1)*qr(2); q(1)*qr(3); q(1)*qr(4)] +...
           [qr(3)*q(4) - qr(4)*q(3); ...
            qr(4)*q(2) - qr(2)*q(4); ...
            qr(2)*q(3) - qr(3)*q(2)];

% Scalar part of quaternion product
scalar_part = qr(1)*q(1) - qr(2)*q(2) - qr(3)*q(3) - qr(4)*q(4);

% Return result of division
q_out = [scalar_part; vec_part];

end