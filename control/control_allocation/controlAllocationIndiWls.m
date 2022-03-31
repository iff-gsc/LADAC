function [ Delta_u, W, iter ] = controlAllocationIndiWls( ca, ...
    B, Delta_nu, u, Delta_u_d, Delta_gamma )
% 
% Inputs:
%   ca                  control allocation parameters struct, see
%                       controlAllocationWlsLoadParams
%   B                   control effectiveness matrix
%   u                   current control input vector
%   Delta_nu            
%   Delta_u_d           
%   Delta_gamma         
%   
% Outputs:
%   Delta_u             optimal incremental control input vector
%   W                   optimal active set
%   iter                number of iterations
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
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2019-2022 First Author
%   Copyright (C) 2022 Second Author
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

gamma = ca.gamma + Delta_gamma;

umin = ca.u_min - u;
umax = ca.u_max - u;
ud = ca.u_d - u + Delta_u_d;
u0 = 0.5 * (umin+umax);

[ Delta_u, W, iter ] = wls_alloc( B, Delta_nu, umin, umax, ...
                    ca.W_v, ca.W_u, ud, gamma, u0, ca.W, ca.i_max );

end
