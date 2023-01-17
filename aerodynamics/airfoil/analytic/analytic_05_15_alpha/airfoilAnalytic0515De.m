function Delta_coeff = airfoilAnalytic0515De(fdcx,alDe)
%airfoilAnalytic0515De is an analytic function for the Delta (De)
% coefficient for angle of attack / alpha due to an actuator state.
%   This analytic function assumes that the "Delta coefficient" can be
%   superpositioned with the coefficient of the clean airfoil.
%   Moreover, it assumes that there is a linear dependency between the
%   actuator state and the Delta coefficient. The proportional factor goes
%   to zero for high angles of attack (tanh function).
% 
% Inputs:
%   fdcx       	analytic function parameters array (see outputs of
%               airfoilAnalytic0515DeFit)
%   alDe        concentrated vectors of angle of attack (first column, in deg)
%               and actuator state (second column) (2xN array)
% 
% Outputs:
%   Delta_coeff     Delta coefficient; to be added to the coefficient of
%                   of the clean airfoil (1xN array)
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

alpha = alDe(1,:);
delta = alDe(2,:);

Delta_coeff = fdcx(1,:).*delta ...
    .* ( 0.5 + 0.5 * tanh( - fdcx(3,:) .* (alpha-fdcx(2,:)) ) );

end