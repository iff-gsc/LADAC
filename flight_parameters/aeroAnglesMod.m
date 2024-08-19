% aeroAnglesMod computes modified angle of attack and angle of sideslip 
% according to an inverted order rotations
%   The modified aerodynamic angles are NOT computed according to 
%   [1, page 78]. The order of rotations from a frame to b frame is 
%   inverted. 
%   According to the ISO 1151 (or LN9300), the rotations from aerodynamic
%   frame (a) to body-fixed frame (b) is -beta about z_a, then alpha
%   about y_b. The inverted rotation is alpha_M about y_a, then -beta
%   about z_b [2, page 72-73]. The use of modified aerodynamic angles is
%   benificial for low airspeed. For small angles, the aerodynamic angles
%   and modified aerodynamic angles are almost similar.
% 
% Literature:
% [1]   Stevens, B. L. et al. (2016): Aircraft Control and Simulation.
%       Dynamics, Control Design, and Autonomous Systems. 3rd ed. Wiley.
% [2]   Beyer, Y. (2016): Flugmechanische Modellierung von Multicopter-
%       Systemen in MATLAB/Simulink. Studienarbeit, TU Braunschweig,
%       unpublished.
% 
% Inputs:
%   V_Ab            a (3xn) matrix with the components of the airspeed
%                   vector in body-fixed frame (b), where n airspeed
%                   vectors can be processed at once
% 
% Outputs:
%   alpha           a (1xn) vector with n modified angles of attack for n 
%                   airspeed vectors
%   beta            a (1xn) vector with n modified sideslip angles for n 
%                   airspeed vectors
% 
% See also: dcmBaFromAeroAnglesMod
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function [ alpha_M, beta_M ] = aeroAnglesMod( V_Ab ) %#codegen

% compute the argument of the asin function
argAlpha = divideFinite( V_Ab(3,:), vecnorm( V_Ab, 2, 1 ) );

% compute the modified angle of attack and avoid complex numbers
alpha_M = asinReal( argAlpha );

% compute the modified sideslip angle
beta_M = atan2( V_Ab(2,:), V_Ab(1,:) );

end
