function [c_L_alpha,alpha0] = airfoilAnalytic0515ClAlphaMax(fcl,varargin)
% airfoilAnalytic0515ClAlphaMax returns the maximum lift curve slope
% (approximately) and the zero lift angle of attack (approximately).
%   Depending on the analytic function coefficients and on the Mach number,
%   the maximum lift curve slope and the zero lift angle of attack are
%   different.
%   As the function is analytic, the maximum value is tried to be
%   determined analytically as well because of efficiency.
%   However, no analytic solution could be found. But the implemented
%   analytic solution returns a value that is very close to the actual
%   value.
% 
% Inputs:
%   fcl         analytic function parameters array (see outputs of
%               airfoilAnalytic0515AlFit)
%   Ma          Mach number (1xN array)
% 
% Outputs:
%   c_L_alpha 	maximum lift curve slope (1xN array), in 1/deg
%   alpha0      zero lift angle of attack (1xN array), in deg
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if isempty(varargin)
    Ma = 0;
else
    Ma = varargin{1};
end

betaMa = 1./sqrtReal(1-powerFast(fcl(5,:).*Ma(:)',2));

f1 = fcl(2,:);

c_L_alpha = f1.*betaMa;

% neglect tanh part
alpha0 = fcl(1,:);
% add one iteration to improve alpha0 accuracy
x = fcl(6,:) .* ( pi/90*(alpha0-fcl(4,:)));
x = x + powerFast(x,3)/3 + powerFast(x,5)/5;
alpha0 = alpha0 ...
    + fcl(3,:) .* ( 1 + tanh( x ) ) ...
    ./ -c_L_alpha;

end