function [ x, w, u, v, Anorm, alfa, rhobar, phibar ] = ... 
    ladacLsqrInit( A, b)

% ladacLsqrInit initializes the auxiliary variables of the LSQR
%   algorithm for the successive iterations on the given system of matrix A 
%   and the right-hand side b.
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
%   b            right-hand side of the system Ax = b
%                (Nx1 vector), dimensionless
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
% See also: ladac_lsqr_iterate, polyInterpolationAx,
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

explicitA = true;

if isa(A,'numeric')
  explicitA = true;
elseif isa(A,'function_handle')
  explicitA = false;
else
  error('SOL:lsqrSOL:Atype','%s','A must be numeric or a function handle');
end

n = length(b);

Anorm  = zeros(1, superiorfloat(b));             

% Set up the first vectors u and v for the bidiagonalization.
% These satisfy  beta*u = b,  alfa*v = A'u.

x    = zeros(n,1, superiorfloat(b));
u    = b;        
v    = zeros(n,1, superiorfloat(b));
w    = zeros(n,1, superiorfloat(b));
alfa = zeros(1, superiorfloat(b));             
beta = norm(u);

rhobar = zeros(1, superiorfloat(b));
phibar = zeros(1, superiorfloat(b));

if beta > 0
   u = (1/beta)*u;
   
   if explicitA
       v = A'*u;
   else
       v = A(u,2);
   end
   
   alfa = norm(v);
end
if alfa > 0
   v = (1/alfa)*v;      w = v;
end

Arnorm = alfa*beta;     

if Arnorm == 0
    return;
end

rhobar = alfa;          phibar = beta;  

end