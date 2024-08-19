function c_m = airfoilAnalytic0515AlCm(fcm,alpha)
% airfoilAnalytic0515AlCm is an analytic function for the pitching moment
% coefficient w.r.t. angle of attack / alpha (Al).
%   It returns good results in for angles of attack between -5 deg and 15
%   deg (0515).
%   This function can compute N airfoils/conditions at once.
% 
% Inputs:
%   fcm         analytic function parameters array (see outputs of
%               airfoilAnalytic0515AlFit)
%   alpha       angle of attack (1xN array), in deg
% 
% Outputs:
%   c_m         pitching moment coefficient (1xN array)
% 
% Literature:
%   [1] https://arc.aiaa.org/doi/pdfplus/10.2514/1.C034910
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

filter = 0.5 + 0.5*tanh( fcm(4,:) .* (pi/90*(alpha-fcm(5,:)) ) );
% own formula
c_m = fcm(1,:) ...
    + fcm(2,:) .* (alpha-fcm(3,:)).^2 .* ( 1 - filter ) ...
    + filter .* fcm(6,:) .* sin(pi/90*alpha);

end