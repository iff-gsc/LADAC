function [bodyState] = wingSetBodyState( bodyState, alpha, beta, V, omega ) %#codegen
%setBodyState sets the current wing state
%   The function setBodyState transferes the values of the input data 
%   (state of the wing) to the objects of the input struct. 
%
% Syntax:  [bodyState] = setBodyState( alpha, beta, V, omega, bodyState )
%
% Inputs:
% 	 alpha                  Angle of attack, in rad
%                           (double)
%    beta                   Sideslip angle, in rad
%                           (double)
%    alpha_dt               time derivative of angle of attack, in rad/s
%                           (double)
%    V                      Air speed, in m/s
%                           (double)
%    omega                  Angular velocity, in rad/s
%                           (array) 
%    rho                    air density (double), in kg/m^3
%
% Outputs:
%    bodyState              Wing body state with tranfered values, in -
%                           (struct)
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% airspeed
bodyState.V = V;

% angular velocity
bodyState.omega = omega;

% angle of attack
bodyState.alpha = alpha;
% sideslip angle
bodyState.beta = beta;

end