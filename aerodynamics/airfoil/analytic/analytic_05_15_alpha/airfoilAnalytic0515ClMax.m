function [c_L_max,alpha_max] = airfoilAnalytic0515ClMax(fcl,varargin)
% airfoilAnalytic0515ClMax returns the maximum lift coefficient
% (approximately) and the corresponding angle of attack of the analytic
% lift curve.
%   Depending on the analytic function coefficients and on the Mach number,
%   the maximum lift coefficient is different.
%   As the function is analytic, the maximum value is tried to be
%   determined analytically as well because of efficiency.
%   However, no analytic solution could be found. But the implemented
%   analytic solution returns a value that is very close to the actual
%   maximum.
%   To Do: The definition of the analytic function was changed:
%       x -> x + x.^3/3 + x.^5/5;
%   This modification yielded a better fit but it makes the computation of
%   the maximum more complicated. This function was not adjusted since that
%   change!
% 
% Inputs:
%   fcl         analytic function parameters array (see outputs of
%               airfoilAnalytic0515AlFit)
%   Ma          Mach number (Nx1 array)
% 
% Outputs:
%   c_L_max     lift coefficient (Nx1 array)
%   alpha_max   angle of attack corresponding to the maximum lift
%               coefficient (Nx1 array), in deg
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if isempty(varargin)
    Ma = zeros(size(fcl,1),1);
else
    Ma = varargin{1};
end

betaM = 1./sqrtReal( 1-powerFast( fcl(:,5).*Ma, 2 ) );

% assume that linear part is really linear and not a sine function (else
% iteration would be required)
c_L_alpha_lin = fcl(:,2).*betaM;

% compute derivative w.r.t. x with wolframalpha.com
%   -a*(1+tanh(b*(x-c)))
% set derivative w.r.t. x equal to d (linear lift curve slope) and solve
% for x
%   solve(-a*b*sech^2(b*(x-c))=d,x)
alpha_max_complex = complex( ...
    - acosh( -1i*sqrt(complex(fcl(:,3))).*sqrt(pi/90*fcl(:,6))./sqrt( c_L_alpha_lin ) ) ...
    + pi/90*fcl(:,6).*fcl(:,4) ...
    ) ...
    ./ ( pi/90*fcl(:,6) );

% if there is no maximum due to stall, set it to 15deg
max_found = abs( imag( alpha_max_complex ) ) < 1e-5;
alpha_max = zeros(size(fcl,1),1);
alpha_max(max_found) = real( alpha_max_complex(max_found) );
alpha_max(~max_found) = 15;

% maximum lift coefficient from corresponding angle of attack
c_L_max = airfoilAnalytic0515AlCl( fcl, [alpha_max, Ma(:)] );
c_L_max(~max_found) = 0.11*alpha_max(~max_found);
end