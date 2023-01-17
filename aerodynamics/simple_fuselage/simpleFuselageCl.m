% simpleFuselageCl computes the lift coefficient of an aircraft fuselage
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
%   C_L_dAlpha0     partial derivative of the lift coefficient w.r.t. the
%                   angle of attack for beta = 0 (scalar), in 1/rad
%   C_Lmax          maximum lift coefficient reached at alpha = 45 degree 
%                   (scalar), in 1
% 
% Outputs:
%   C_L             lift coefficient (scalar, vector or matrix), in 1
% 
% Example:
%   alpha_M             = -pi/2:0.05:pi/2;
%   beta_M              = -pi:0.05:pi;
%   [ Alpha_M, Beta_M ] = meshgrid( alpha_M, beta_M );
%   C_L                 = simpleFuselageCl( Alpha_M, Beta_M, 0.5, 0.3 );
%   figure
%   surf( Alpha_M, Beta_M, C_L )
%   xlabel('\alpha_M'), ylabel('\beta_M'), zlabel('C_L')
% 
% See also: aeroAnglesMod, simpleFuselageCq
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function C_L = simpleFuselageCl( alpha_M, beta_M, C_L_dAlpha0, C_Lmax ) %#codegen


    % lift coefficient for beta = 0
    C_L_beta0 = 0.5 * C_L_dAlpha0 * sin( 2*alpha_M ) ...
        + 0.5 * ( C_Lmax - 0.5*C_L_dAlpha0 ) * ( 1 - cos( 4*alpha_M ) ) ...
        .* sign(alpha_M);

    % reduce pitching moment for increasing sideslip angle
    C_L = C_L_beta0 .* cos( beta_M ).^2;

end