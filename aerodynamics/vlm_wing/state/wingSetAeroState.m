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

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
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
    error('Online update of AIC currently not supported.');
else
    V_Ab(:) = dcmBaFromAeroAngles( wing.state.body.alpha, wing.state.body.beta ) ...
        * [ wing.state.body.V; 0; 0 ];
    [~,beta_rot(:)] = aeroAngles( wing.interim_results.M_rot_x * V_Ab );

    if sign(beta_rot) > 0
        wing.interim_results.AIC_b = ...
            wing.interim_results.AIC_b + ...
            wing.interim_results.Delta_AIC_b_pos ...
            * abs(beta_rot) / wing.interim_results.beta_infl;
        wing.interim_results.AIC_t = ...
            wing.interim_results.AIC_t + ...
            wing.interim_results.Delta_AIC_t_pos ...
            * abs(beta_rot) / wing.interim_results.beta_infl;
    else
        wing.interim_results.AIC_b = ...
            wing.interim_results.AIC_b + ...
            wing.interim_results.Delta_AIC_b_neg ...
            * abs(beta_rot) / wing.interim_results.beta_infl;
        wing.interim_results.AIC_t = ...
            wing.interim_results.AIC_t + ...
            wing.interim_results.Delta_AIC_t_neg ...
            * abs(beta_rot) / wing.interim_results.beta_infl;
    end
end

% switch wing.config.method
%     case 'IVLM'
        % compute circulation and angle of attack
        wing = wingSetCirculationUnsteady( wing );
%     case 'DLM'
%         wing = wingSetCirculationDlm( wing );
% end

% compute force coefficient distribution
wing = wingSetLocalCoeff( wing );

% compute global force coefficients
wing = wingSetGlobalCoeff( wing );

% compute local forces and moments
wing = wingSetLocalForce( wing );

% compute global force and moments
wing = wingSetGlobalForce( wing, pos_ref_c );

end