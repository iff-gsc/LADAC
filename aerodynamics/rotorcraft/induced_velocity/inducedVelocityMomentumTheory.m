function [lambda_i] = inducedVelocityMomentumTheory(Vx, Vz) %#codegen
% inducedVelocityMomentumTheory computes the induced velocity of a rotor.
%
%   The computation is based on the momentum theory. The equation is
%   solved according to [1, page 153]. The computation is non conducted
%   with the dimensionless lambda but with absolute velocities.
%   This function does not consider vortex ring state.
%
% Literature:
% [1]   Van der Wall, B. G. (2015): Grundlagen der Hubschrauber-
%       Aerodynamik. Springer.
%
% Syntax: [v_i] = inducedVelocityMomentumTheory(V_x, V_z, v_h) 
%
% Inputs:
%   Vx         forward velocity of the rotor plane relative to the air
%               (scalar), in m/s
%   Vz         velocity of the propeller relative to the air perpendicular
%               to the rotor. Positive V_z means rotor is ascending,
%               negative V_z means that the rotor is descending.
%               (scalar), in m/s                              
%
% Outputs:
%   lambda_i   induced velocity of the propeller (scalar), in m/s
%
% See also: InducedVelocity
%

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

lambda_h   = 1 ;
lambda_h_2 = 1;
mu         = abs(Vx) ;
mu_z       = Vz;
lambda_n   = lambda_h;

% mu_z give information about the operationg point of the rotor
Vz_div_vh = mu_z;

% % Make sure to operate outside the vortex ring state (-2 < Vz_div_vh <- 1)
% if ( -2 < Vz_div_vh) && (Vz_div_vh < -1.5) 
%     error('MomentumTheory:VortexRingState','Operating in Vortex Ring State. Please check (-2 < V_z/v_h <- 1)')
% end

if(Vz_div_vh <= -1.5)
    
    % Fixpoint-Iteration for fast vertival descent [1, p.152]
    % Newton Iteration seem to have problems around (V_z/v_h) = -2.
    
    for k = 1:50
        
        lambda_n = mu_z + lambda_h_2 / sqrt(mu^2 + lambda_n^2 );
        
    end
    
else
    
% Iteration for vertival flight and climb [1, p.153]
    
    for k = 1:20
        
        f_lambda = lambda_n - mu_z - lambda_h_2 / sqrt(mu^2 + lambda_n^2 );
        
        f_lambda_derivative = 1 + lambda_n * lambda_h_2 / ...
                              (sqrt(mu^2 + lambda_n^2 ))^3;
        
        lambda_n = lambda_n - (f_lambda / f_lambda_derivative);
    end
    
end

% induced velocity lambda_i = lambda - mu_z
lambda_i = lambda_n - mu_z;


end

