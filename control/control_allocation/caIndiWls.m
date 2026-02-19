function [ Delta_u, W, iter ] = caIndiWls( ca, ...
    B, Delta_nu, u, varargin )
% caIndiWls adapt WLS control allocation for INDI
%   The adaption of WLS control allocation for INDI is described in [1].
% 
% Syntax:
%   [Delta,W,iter] = caIndiWls(ca,B,Delta_nu,u)
%   [Delta,W,iter] = caIndiWls(ca,B,Delta_nu,u,Name,Value)
% 
% Inputs:
%   ca                  Control allocation parameters struct, see
%                       controlAllocationWlsLoadParams
%   B                   Control effectiveness matrix (NxM array)
%   Delta_nu            Incremental pseudo-control input vector (Nx1 array)
%   u                   Current control input vector (Mx1 array)
%   Name                The following optional 'Name' arguments can be
%                       passed (string) followed by Value:
%                           'DeltaUd'       followed by Delta_u_d
%                           'DeltaGamma'    followed by Delta_gamma
%                           'DeltaDiagWv'   followed by Delta_diag_W_v
%                           'DeltaUmax'     followed Delta_u_max
%   Delta_u_d           (optional) Online adjustment of desired u (scalar
%                       or Mx1 array); default: 0
%   Delta_gamma         (optional) Online adjustment of ca.gamma (scalar);
%                       default: 0
%   Delta_diag_W_v      (optinal) Online adjustment of ca.W_v diagonal
%                       (scalar or Nx1 array); default: 0
%   Delta_u_max         (optional) Maximum control increment (scalar or Mx1
%                       array); default: abs(ca.u_max-ca.u_min)
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

Delta_u_d       = zeros( size(ca.u_d) );
Delta_gamma     = 0;
Delta_diag_W_v  = zeros( size(Delta_nu) );
Delta_u_max     = abs(ca.u_max-ca.u_min);
for i = 1:length(varargin)
    if strcmp(varargin{i},'DeltaUd')
        Delta_u_d(:) = varargin{i+1};
    elseif strcmp(varargin{i},'DeltaGamma')
        Delta_gamma(:) = varargin{i+1};
    elseif strcmp(varargin{i},'DeltaDiagWv')
        Delta_diag_W_v(:) = varargin{i+1};
    elseif strcmp(varargin{i},'DeltaUmax')
        Delta_u_max(:) = varargin{i+1};
    end    
end

% adjustments according to [1]
umin    = ca.u_min - u;
umax    = ca.u_max - u;
ud      = ca.u_d - u;
u0      = 0.5 * (umin+umax);

% apply u rate limit
umin    = max( umin, -Delta_u_max );
umax    = min( umax, Delta_u_max );

% adjustments of online WLS parameter changes
gamma   = ca.gamma + Delta_gamma;
ud      = ud + Delta_u_d;

if ~all(size(ca.W_v) > 1)
    W_v = diag( ca.W_v );
else
    W_v = ca.W_v;
end
W_v     = W_v + diag( Delta_diag_W_v );

if ~all(size(ca.W_u) > 1)
    W_u = diag( ca.W_u );
else
    W_u = ca.W_u;
end

% not used yet, dummy values
W       = zeros( length(ca.W_u), 1, superiorfloat(ca.W_u) );

[ Delta_u, W, iter ] = wls_alloc( B, Delta_nu, umin, umax, ...
    W_v, W_u, ud, gamma, u0, W, ca.i_max );

end
