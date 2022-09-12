function [ n_b_upset, n_b_dt_upset, n_b_dt2_upset, nu_n_b_dt2_upset ] = ...
    indiCopterLeanVectorUpset( n_b, n_b_dt, n_b_dt2, nu_n_b_dt2 )
% indiCopterLeanVectorUpset modify lean vector for upset recovery
%   Using the lean vector for reduced attitude control of multicopters does
%   not work in case the desired lean vector points into negative current
%   thrust direction. Therefore, the lean vector and its time derivatives
%   are modified such that the control error further increases in case of a
%   tilt error greater than 90 degrees.
% 
% Syntax:
%   [ n_b_upset, n_b_dt_upset, n_b_dt2_upset, nu_n_b_dt2_upset ] = ...
%       indiCopterLeanVectorUpset( n_b, n_b_dt, n_b_dt2, nu_n_b_dt2 )
% 
% Inputs:
%   n_b             lean vector (3x1 array) represented in body-fixed frame
%                   (b), dimensionless
%   n_b_dt          time derivative of input n_b
%   n_b_dt2         second time derivative of input n_b
%   nu_n_b_dt2      pseudo control input n_b_dt2
% 
% Outputs:
%   n_b_upset           modified lean vector (3x1 array)
%   n_b_dt_upset        time derivative out output n_b_upset
%   n_b_dt2_upset       second time derivative of output n_b_upset
%   nu_n_b_dt2_upset    modified pseudo control input n_b_dt2
% 
% See also:
%   dcm2LeanVector, indiCopterAcc2LeanVector, leanVectorDerivTrafo

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

n_b_upset           = n_b;
n_b_dt_upset        = n_b_dt;
n_b_dt2_upset       = n_b_dt2;
nu_n_b_dt2_upset	= nu_n_b_dt2;

if n_b(3) > 0
    if n_b(3) > 0.99
        % avoid singularity if exactly upset
        n_xy            = [0;1];
    else
        vec_xy          = n_b(1:2);
        n_xy            = divideFinite( vec_xy, norm( vec_xy, 2 ) );
    end
    n_b_upset(1:2)      = 2*n_xy - n_b(1:2);
    n_b_dt_upset(1:2)	= -n_b_dt(1:2);
    n_b_dt2_upset(1:2)	= -n_b_dt2(1:2);
end

end