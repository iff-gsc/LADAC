function [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ...
    ladac_lsqr_iterate( A, x, w, u, v, Anorm, alfa, rhobar, phibar)

% ladac_lsqr_iterate executes one step of the LSQR algorithm
%   The function executes the LSQR algorithm for a single iteration
%   on the given system of matrix A and the intermediate results
%   from the last step or initalization.
%
%   Note:
%   This is a very special function for solving a linear system of
%   equations related to polynomial interpolation.
%   This function is not intended to replace a standard solver for linear
%   equation systems, since there are no termination criteria apart from
%   the number of iterations.
%
% Inputs:
%
%   A            matrix A of the system Ax = b
%                (NxN vector), dimensionless
%
%   x            auxiliary vector from the last calculation
%                (Nx1 vector), dimensionless
%
%   w            auxiliary vector from the last calculation
%                (Nx1 vector), dimensionless
%
%   u            auxiliary vector from the last calculation
%                (Nx1 vector), dimensionless
%
%   v            auxiliary vector from the last calculation
%                (Nx1 vector), dimensionless
%
%   Anorm        scalar auxiliary quantity from the last calculation
%                (scalar), dimensionless
%
%   alfa         scalar auxiliary quantity from the last calculation
%                (scalar), dimensionless
%
%   rhobar       scalar auxiliary quantity from the last calculation
%                (scalar), dimensionless
%
%   phibar       scalar auxiliary quantity from the last calculation
%                (scalar), dimensionless
%
% Outputs:
%
%   x            auxiliary vector from this iteration
%                (Nx1 vector), dimensionless
%
%   w            auxiliary vector from this iteration
%                (Nx1 vector), dimensionless
%
%   u            auxiliary vector from this iteration
%                (Nx1 vector), dimensionless
%
%   v            auxiliary vector from this iteration
%                (Nx1 vector), dimensionless
%
%   Anorm        scalar auxiliary quantity from this iteration
%                (scalar), dimensionless
%
%   alfa         scalar auxiliary quantity from this iteration
%                (scalar), dimensionless
%
%   rhobar       scalar auxiliary quantity from this iteration
%                (scalar), dimensionless
%
%   phibar       scalar auxiliary quantity from this calculation step
%                (scalar), dimensionless
%
% Syntax:
%   [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ...
%   ladac_lsqr_iterate( A, x, w, u, v, Anorm, alfa, rhobar, phibar)
%
% See also: polyInterpolationb, polyInterpolationAx,
%           trajFromWaypointsIterative, polyInterpolationCore
%
% Literature:
%   [1] C. C. Paige and M. A. Saunders (1982a).
%       LSQR: An algorithm for sparse linear equations and sparse least squares,
%       ACM TOMS 8(1), 43-71.
%   [2] C. C. Paige and M. A. Saunders (1982b).
%       Algorithm 583.  LSQR: Sparse linear equations and least squares problems,
%       ACM TOMS 8(2), 195-209.
%   [3] M. A. Saunders (1995).  Solution of sparse rectangular systems using
%       LSQR and CRAIG, BIT 35, 588-604.

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
%
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

damp = 0;
explicitA = true;

if isa(A,'numeric')
    explicitA = true;
elseif isa(A,'function_handle')
    explicitA = false;
else
    error('SOL:lsqrSOL:Atype','%s','A must be numeric or a function handle');
end

%------------------------------------------------------------------
%     Main iteration loop.
%------------------------------------------------------------------

% Perform the next step of the bidiagonalization to obtain the
% next beta, u, alfa, v.  These satisfy the relations
%      beta*u  =  A*v  - alfa*u,
%      alfa*v  =  A'*u - beta*v.

if explicitA
    u = A*v    - alfa*u;
else
    u = A(v,1) - alfa*u;
end

beta = norm(u);
if beta > 0
    u     = (1/beta)*u;
    Anorm = norm([Anorm alfa beta damp]);
    
    if explicitA
        v = A'*u   - beta*v;
    else
        v = A(u,2) - beta*v;
    end
    
    alfa  = norm(v);
    
    if alfa > 0
        v = (1/alfa)*v;
    end
    
end

% Use a plane rotation to eliminate the damping parameter.
% This alters the diagonal (rhobar) of the lower-bidiagonal matrix.

rhobar1 = norm([rhobar damp]);
cs1     = rhobar/rhobar1;
phibar  = cs1*phibar;

rho     =   norm([rhobar1 beta]);
cs      =   rhobar1/rho;
sn      =   beta   /rho;
theta   =   sn*alfa;
rhobar  = - cs*alfa;
phi     =   cs*phibar;
phibar  =   sn*phibar;

t1      =   phi  /rho;
t2      = - theta/rho;

x       = x      + t1*w;
w       = v      + t2*w;

end
