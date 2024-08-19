% simpleFuselageCd computes the drag coefficient of an aircraft fuselage
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
%   C_Dmin          minimum drag coefficient at alpha = beta = 0 (scalar),
%                   in 1/rad
%   C_Dmax          maximum drag coefficient at alpha = 90 degree (scalar),
%                   in rad
% 
% Outputs:
%   C_D             drag coefficient (scalar, vector or matrix), in 1
% 
% Example:
%   alpha_M             = -pi/2:0.05:pi/2;
%   beta_M              = -pi:0.05:pi;
%   [ Alpha_M, Beta_M ] = meshgrid( alpha_M, beta_M );
%   C_D                 = simpleFuselageCd( Alpha_M, Beta_M, 0.1, 0.3 );
%   figure
%   surf( Alpha_M, Beta_M, C_D )
%   xlabel('\alpha_M'), ylabel('\beta_M'), zlabel('C_D')
% 
% See also: aeroAnglesMod
% 

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function C_D = simpleFuselageCd( alpha_M, beta_M, C_Dmin, C_Dmax ) %#codegen


    % delta drag coefficient for beta = 0
    Delta_C_D_beta0 = C_Dmax/2 * ( 1 - cos(2*alpha_M) );

    % delta drag coefficient for alpha = 0
    Delta_C_D_alpha0 = C_Dmax/2 * ( 1 - cos(2*beta_M) );

    % compute total drag coefficient
    C_D = Delta_C_D_beta0 + Delta_C_D_alpha0.*cos(alpha_M).^2 + C_Dmin;

end