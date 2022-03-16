function c_D = airfoilAnalytic0515AlCd(fcd,alpha)
% airfoilAnalytic0515AlCd is an analytic function for the drag coefficient
% w.r.t. angle of attack / alpha (Al).
%   It returns good results in for angles of attack between -5 deg and 15
%   deg (0515).
%   This function can compute N airfoils/conditions at once.
% 
% Inputs:
%   fcd         analytic function parameters array (see outputs of
%               airfoilAnalytic0515AlFit)
%   alpha       angle of attack (Nx1 array), in deg
% 
% Outputs:
%   c_D         drag coefficient (Nx1 array)
% 
% Literature:
%   [1] https://arc.aiaa.org/doi/pdfplus/10.2514/1.C034910

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

filter = 0.5 + 0.5*tanh( fcd(:,4) .* (pi/90*(alpha-fcd(:,5)) ) );
% own formula
c_D = fcd(:,1) ...
    + fcd(:,2) .* (alpha-fcd(:,3)).^2 .* ( 1 - filter ) ...
    + filter .* fcd(:,6) .* sin(pi/180*alpha);

end