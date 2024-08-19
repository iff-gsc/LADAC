function [x] = ladacLsqr( A, b, itnlim)
% ladac_lsqr solves the linear equation system Ax = b
%   The function executes the LSQR algorithm for a fixed number of given
%   iterations on the given system of matrix A and right-hand side b.
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
%   b            right-hand side b of the system Ax = b
%                (Nx1 vector), dimensionless
%
%   itnlim       number of iterations
%                (scalar)
%
% Outputs:
%
%   x            solution of the system Ax = b
%                (Nx1 vector), dimensionless
%
% Syntax:
%   [coeffs, num_of_splines, degree] = ...
%   polyInterpolationIterative(points, degree, cycle, plot_enable, ...
%   derivatives)
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

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
%
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

m = size(A,1);
n = size(A,2);

damp  = 0;
itn   = 0;
Anorm = 0;

% Set up the first vectors u and v for the bidiagonalization.
% These satisfy  beta*u = b,  alfa*v = A'u.

x    = zeros(n,1);
u    = b(1:m);
alfa = 0;
beta = norm(u);

if beta > 0
    u = (1/beta)*u;
    v = A'*u;
    alfa = norm(v);
end

if alfa > 0
    v = (1/alfa)*v;
    w = v;
end

Arnorm = alfa*beta;

if Arnorm == 0
    return;
end

rhobar = alfa;
phibar = beta;

%------------------------------------------------------------------
%     Main iteration loop.
%------------------------------------------------------------------
while itn < itnlim
    itn = itn + 1;
    
    % Perform the next step of the bidiagonalization to obtain the
    % next beta, u, alfa, v.  These satisfy the relations
    %      beta*u  =  A*v  - alfa*u,
    %      alfa*v  =  A'*u - beta*v.
    
    u = A*v    - alfa*u;
    
    beta = norm(u);
    if beta > 0
        u     = (1/beta)*u;
        Anorm = norm([Anorm alfa beta damp]);
        
        v = A'*u   - beta*v;
        
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
end

