function q_out = quatMultiply(q, p)
% quatMultiply multiply two quaternions
%   q_out = quatMultiply( q ) computes the multiplication of two 
%   quaternions in the form of [w; x; y; z]. With the scalar part w and
%   vector parts x, y and z.
% 
% Syntax:
%   q_out = quatMultiply( q, qr )
% 
% Inputs:
%   q               quaternion (4x1 array), dimensionless
%   qr              quaternion (4x1 array), dimensionless
% 
% Outputs:
%   q_out           result of multiplication  (4x1 array), dimensionless
%                    
% Example: 
%   q  = [0.7071; 0.7071; 0; 0];
%   qr = [0.7071; -0.7071; 0; 0];
%   q_out = quatMultiply(q, qr)
% 
% See also:
%   quatDivide, quatInv, quatNorm
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%Vector part of quaternion product
vec_part = [q(1)*p(2); q(1)*p(3); q(1)*p(4)] + ...
           [p(1)*q(2); p(1)*q(3); p(1)*q(4)] +...
           [q(3)*p(4) - q(4)*p(3); ...
            q(4)*p(2) - q(2)*p(4); ...
            q(2)*p(3) - q(3)*p(2)];

% Scalar part of quaternion product
scalar_part = q(1)*p(1) - q(2)*p(2) - q(3)*p(3) - q(4)*p(4);

% Return result of multiplication
q_out = [scalar_part; vec_part];

% qw = q(1);
% qv = q(2:4);
% 
% pw = p(1);
% pv = p(2:4);
% 
% q_out = [qw*pw-dot(qv,pv); cross(qv, pv)+qw*pv+pw*qv];

end