% simpleFuselageCq computes the side force coefficient of an aircraft fuselage
%   using a simple generic model. The model considers the modified 
%   aerodynamic angles as well as two parameters. The model is documented
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
%   C_Q_dBeta0      partial derivative of the side force coefficient w.r.t.
%                   the sideslip angle for alpha = 0 (scalar), in 1/rad
%   C_Qmax          maximum side force coefficient reached at beta = 45
%                   degree (scalar), in 1
% 
% Outputs:
%   C_Q             side force coefficient (scalar, vector or matrix), in 1
% 
% Example
%   alpha_M             = -pi/2:0.05:pi/2;
%   beta_M              = -pi:0.05:pi;
%   [ Alpha_M, Beta_M ] = meshgrid( alpha_M, beta_M );
%   C_Q                 = simpleFuselageCq( Alpha_M, Beta_M, 0.2, 0.6 );
%   figure
%   surf( Alpha_M, Beta_M, C_Q )
%   xlabel('\alpha_M'), ylabel('\beta_M'), zlabel('C_Q')
% 
% See also: aeroAnglesMod, simpleFuselageCl
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function C_Q = simpleFuselageCq( alpha_M, beta_M, C_Q_dBeta0, C_Qmax ) %#codegen

    % side force coefficient for alpha = 0
    C_Q_alpha0 = 0.5 * C_Q_dBeta0 * sin(2*beta_M) ...
        + 0.5 * ( -C_Qmax - 0.5*C_Q_dBeta0 ) * ( 1 - cos(4*beta_M) ) ...
        .* sign( sin(2*beta_M) );

    % reduce side force coefficient for increasing angle of attack
    C_Q = C_Q_alpha0 .* cos(alpha_M).^2;
    
end