function [ Delta_u, W, iter ] = caIndiWls( ca, ...
    B, Delta_nu, u, Delta_u_d, Delta_gamma, Delta_diag_W_v )
% caIndiWls adapt WLS control allocation for INDI
%   The adaption of WLS control allocation for INDI is described in [1].
% 
% Inputs:
%   ca                  control allocation parameters struct, see
%                       controlAllocationWlsLoadParams
%   B                   control effectiveness matrix (NxM array)
%   Delta_nu            Incremental pseudo-control input vector (Nx1 array)
%   u                   current control input vector (Mx1 array)
%   Delta_u_d           online adjustment of desired u (scalar or Mx1
%                       array); should be 0 if not needed.
%   Delta_gamma         online adjustment of ca.gamma (scalar); should be 0
%                       if not needed.
%   Delta_diag_W_v      online adjustment of ca.W_v diagonal (scalar or 
%                       Nx1 array); should be 0 if not needed
%   
% Outputs:
%   Delta_u             optimal incremental control input vector (Mx1
%                       array)
%   W                   optimal active set (Mx1 array)
%   iter                number of iterations (scalar)
% 
% Literature:
%   [1] Smeur, E., Höppener, D., & Wagter, C. D. (2017). Prioritized
%       Control Allocation for Quadrotors Subject to Saturation, In H. D. 
%       P. J.-M. Moschetta, G. Hattenberger (Ed.), International Micro Air
%       Vehicle Conference and Flight Competition 2017 (pp. 37-43).
%       Toulouse, France.
% 
% See also:
%   wls_alloc

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% adjustments according to [1]
umin    = ca.u_min - u;
umax    = ca.u_max - u;
ud      = ca.u_d - u;
u0      = 0.5 * (umin+umax);

% adjustments of online WLS parameter changes
gamma   = ca.gamma + Delta_gamma;
ud      = ud + Delta_u_d;
W_v     = ca.W_v + diag( Delta_diag_W_v );

datatype = superiorfloat(ca.u_min);

% init
Delta_u = zeros(size(ca.u_d),datatype);
W       = zeros(size(Delta_nu),datatype);
iter    = zeros(1,datatype);

% run WLS
if ca.force_double_precision
    [ Delta_u_double, W_double, iter_double ] = wls_alloc( ...
        double(B), double(Delta_nu), double(umin), double(umax), ...
        double(W_v), double(ca.W_u), double(ud), double(gamma), ...
        double(u0), double(ca.W), double(ca.i_max) );
    if strcmp(datatype,'single')
        Delta_u(:) = single(Delta_u_double);
        W(:)       = single(W_double);
        iter(:)    = single(iter_double);
    else
        Delta_u(:) = Delta_u_double;
        W(:)       = W_double;
        iter(:)    = iter_double;
    end
else
    [ Delta_u(:), W(:), iter(:) ] = wls_alloc( B, Delta_nu, umin, umax, ...
        W_v, ca.W_u, ud, gamma, u0, ca.W, ca.i_max );
end

end
