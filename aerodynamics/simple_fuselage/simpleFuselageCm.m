% simpleFuselageCm computes the pitchin moment coefficient of an aircraft 
%   fuselage using a simple generic model. The model considers the modified 
%   aerodynamic angles as well as one parameters. The model is documented
%   in [1, p. 29f] and it is based on measured data presented in 
%   [2, p 248].
% 
% Literature:
%   [1] Beyer, Y. (2017): Flight Control Design and Simulation of a Tandem
%       Tilt Wing RPAS, master thesis, TU Braunschweig.
%   [2] Schlichting, H. & Truckenbrodt, E. (2001): Aerodynamik des
%       Flugzeuges - Teil 2, Springer.
% 
% Inputs:
%   alpha_M         modified angle of attack (scalar, vector or matrix), in
%                   rad
%   beta_M          modified sideslip angle (scalar, vector or matrix), in
%                   rad
%   C_m_dAlpha0     partial derivative of the pitching moment coefficient 
%                   w.r.t. the angle of attack for beta = 0 (scalar), in 
%                   1/rad
% 
% Outputs:
%   C_m             pitching moment coefficient (scalar, vector or matrix),
%                   in 1
% 
% Example:
%   alpha_M             = -pi/2:0.05:pi/2;
%   beta_M              = -pi:0.05:pi;
%   [ Alpha_M, Beta_M ] = meshgrid( alpha_M, beta_M );
%   C_m                 = simpleFuselageCm( Alpha_M, Beta_M, 0.5 );
%   figure
%   surf( Alpha_M, Beta_M, C_m )
%   xlabel('\alpha_M'), ylabel('\beta_M'), zlabel('C_m')
% 
% See also: aeroAnglesMod, simpleFuselageCl, simpleFuselageCn
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function C_m = simpleFuselageCm( alpha_M, beta_M, C_m_dAlpha0 ) %#codegen

    % pitching moment coefficient for beta = 0
    C_m_beta_0 = 0.5 * C_m_dAlpha0 * sin(2*alpha_M);

    % reduce pitching moment for increasing sideslip angle
    C_m = C_m_beta_0 .* cos( beta_M ).^2;

end