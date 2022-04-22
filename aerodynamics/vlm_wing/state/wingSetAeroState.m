function wing = wingSetAeroState( wing, pos_ref_c )
% wingSetAeroState computes the aerodynamic state of the wing.
% 
% Inputs:
% 	wing           	wing struct, see wingCreate
%   pos_ref_c      	vehicle reference position (for rigid body parameters)
%                   in c frame (3x1 array), in m
% 
% Outputs:
% 	wing           	wing struct, see wingCreate
% 
% See also: wingCreateState, wingCreate, wingSetCirculationUnsteady

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% set local inflow
wing = wingSetLocalInflow( wing, pos_ref_c );

% dimensionless induced velocities
beta_rot = 0;
V_Ab = zeros(3,1);
if wing.config.is_infl_recomputed
    wing.interim_results.dimless_induced_vel_beta = ...
        wingGetDimlessIndVel( -wing.state.aero.local_inflow.V, wing.state.geometry );
else
    V_Ab(:) = dcmBaFromAeroAngles( wing.state.body.alpha, wing.state.body.beta ) ...
        * [ wing.state.body.V; 0; 0 ];
    [~,beta_rot(:)] = aeroAngles( wing.interim_results.M_rot_x * V_Ab );

    if sign(beta_rot) > 0
        wing.interim_results.dimless_induced_vel_beta = ...
            wing.interim_results.dimless_induced_vel + ...
            wing.interim_results.Delta_dimless_induced_vel_pos ...
            * abs(beta_rot) / wing.interim_results.beta_infl;
    else
        wing.interim_results.dimless_induced_vel_beta = ...
            wing.interim_results.dimless_induced_vel + ...
            wing.interim_results.Delta_dimless_induced_vel_neg ...
            * abs(beta_rot) / wing.interim_results.beta_infl;
    end
end

% compute circulation and angle of attack
wing = wingSetCirculationUnsteady( wing );

% compute force coefficient distribution
wing = wingSetLocalCoeff( wing );

% compute global force coefficients
wing = wingSetGlobalCoeff( wing );

end