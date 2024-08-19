function c_L = airfoilAnalytic0515AlCl(fcl,alMa)
% airfoilAnalytic0515AlCl is an analytic function for the lift coefficient
% w.r.t. angle of attack / alpha (Al).
%   It returns good results in for angles of attack between -5 deg and 15
%   deg (0515).
%   This function can compute N airfoils/conditions at once.
% 
% Inputs:
%   fcd         analytic function parameters array (see outputs of
%               airfoilAnalytic0515AlFit)
%   alMa        concentrated angle of attack and Mach number (2xN array), angle
%               of attack in deg
% 
% Outputs:
%   c_L   	lift coefficient (1xN array)
% 
% Literature:
%   [1] https://arc.aiaa.org/doi/pdfplus/10.2514/1.C034910

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

alpha = alMa(1,:);
Ma = alMa(2,:);

betaM = 1./sqrtReal( 1-powerFast( Ma, 2 ) );

x = fcl(6,:) .* ( pi/90*(alpha-fcl(4,:)));
% modification of the model in [1]
x = x + powerFast(x,3)/3 + powerFast(x,5)/5;
% [1], page 3
c_L = fcl(2,:) .* betaM.*90/pi.*sin(pi/90*(alpha-fcl(1,:))) ...
    + fcl(3,:) .* ( 1 + tanh( x ) );

end