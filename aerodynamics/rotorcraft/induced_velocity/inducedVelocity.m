function [lambda_i] = inducedVelocity(C_T, mu, mu_z)
% InducedVelocity computes the induced flow ratio of a rotor.
%
%   The computation is based on the momentum theory. The equation is
%   solved according to [1, page 153]. 
%   This function eliminates the singularity of momentum theory in
%   vortex ring state. This Vortex Ring State model uses third-order
%   polynomials, that connects stepwise the two branches of climb and
%   fast decent. [2, page 14]. This model is based on real rotor test data.
%   This model creates an instability for the vortex ring state. Within
%   the defined boundary the real part of the eigenvalue of heave damping
%   become positive.
%   
% Literature:
% [1]   Van der Wall, B. G. (2015): Grundlagen der Hubschrauber-
%       Aerodynamik. Springer.
% 
% [2]   Johnson, Wayne (2005): Model for Vortex Ring State Influence on
%       Rotorcraft Flight Dynamics. NASA Ames Research Center.
%
% Syntax: [lambda_i] = InducedVelocity(C_T, mu, mu_z) 
%
% Inputs:
%   C_T         rotor thrust coefficient, thrust coefficient,
%               C_T = T / rho * A * (omega*R)^2
%               (scalar), dimensionless
%   mu          rotor advance ratio
%               mu = V * cos(alpha) / (omega*R)
%               (scalar), in dimensionless
%   mu_z        climb inflow ratio, 
%               Positive mu_z means rotor is ascending,
%               negative mu_z means that the rotor is descending.
%               (scalar), in dimensionless                           
%
% Outputs:
%   lambda_i    induced inflow ratio
%               (scalar), dimensionless
%
% See also: inducedVelocityWithUnits
%

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Fabian Guecker
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% induced velocity in hover
sign_lambda_h = sign(C_T);
lambda_h = sqrt( abs(C_T) / 2 );
lambda_h = lambda_h * sign_lambda_h;

% check for division by zero 
if(abs(lambda_h) < 1e-8 )
       
    % No calculation needed!
    lambda_i = lambda_h;
    
else
    % calculate forward velocity scaled with induced velocity in hover
    Vx = mu / lambda_h;
    
    % climb velocity scaled with induced velocity in hover
    Vz = mu_z / lambda_h;
    
    %calculate ratio of induced velocity to induced velocity in hover
    vi_vh = inducedVelocityVortexRingState(Vx, Vz);
    
    % scale induced velocity with 
    lambda_i = vi_vh * lambda_h;
end


end
