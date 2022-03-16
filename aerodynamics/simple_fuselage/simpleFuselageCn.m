% simpleFuselageCn computes the yawing moment coefficient of an aircraft 
% fuselage using a simple generic model. The model considers the modified 
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
%   C_n_dBeta0      partial derivative of the yawing moment coefficient 
%                   w.r.t. the sideslip angle for alpha = 0 (scalar), in 
%                   1/rad
% 
% Outputs:
%   C_n             yawing moment coefficient (scalar, vector or matrix), 
%                   in 1
% 
% Example:
%   alpha_M             = -pi/2:0.05:pi/2;
%   beta_M              = -pi:0.05:pi;
%   [ Alpha_M, Beta_M ] = meshgrid( alpha_M, beta_M );
%   C_n                 = simpleFuselageCn( Alpha_M, Beta_M, 0.5 );
%   figure
%   surf( Alpha_M, Beta_M, C_n )
%   xlabel('\alpha_M'), ylabel('\beta_M'), zlabel('C_n')
% 
% See also: aeroAnglesMod, simpleFuselageCq, simpleFuselageCm
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

function C_n = simpleFuselageCn( alpha_M, beta_M, C_n_dBeta0 ) %#codegen


    % yawing moment coefficient for alpha = 0
    C_n_alpha0 = 0.5 * C_n_dBeta0 * sin(2*beta_M);
    
    % reduce yawing moment coefficient for increasing alpha
    C_n = C_n_alpha0 .* cos(alpha_M).^2;

end