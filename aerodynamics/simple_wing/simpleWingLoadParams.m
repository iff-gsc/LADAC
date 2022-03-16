function wing = simpleWingLoadParams( filename )

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% load parameters from file
run(filename);

% add wing's dihedral angle
if ~isfield(wing.geometry, 'v')
    wing.geometry.v = 0;
end

% add potentially missing flap scaling parameters (deactivate scaling by
% default)
if ~isfield(wing.flap,'eff_scaling')
    wing.flap.eff_scaling = 1;
    wing.flap.is_eff_scaling = 0;
end

% compute wing reference surface
wing.geometry.S = wing.geometry.c * wing.geometry.b;

% set additional aerodynamics parameters from geometry
wing = simpleWingSetLiftCurveSlope( wing );
wing = simpleWingSetInducedDragFactor( wing );

% get aerodynamic polar map
wing = simpleWingSetLiftCurve( wing );
wing = simpleWingSetDragCurve( wing );

% x position of the neutral point in the wing frame, m
wing = simpleWingSetNeutralPoint( wing );

% y position shift of the neutral point of the left wing side and the right
% wing side in wing frame if beta=90deg, m
wing.delta_y_np_a0b90_b = 0.90*wing.geometry.b/2;
wing.delta_x_np_a0b90_b = 0;

% compute elevon/flap effectiveness
wing = simpleWingSetFlapEffectiveness( wing );

% compute control point location for local airspeed vector computation
spanwise_location = 0.65;
wing.xyz_cntrl_pt_wing = [ ...
    [1,1] * -spanwise_location * wing.geometry.b/2 * tan(wing.geometry.phi) ...
        - wing.geometry.c/2 * ( (1+wing.geometry.z)-2*wing.geometry.z*spanwise_location ); ...
    [-1,1] * spanwise_location * wing.geometry.b/2; ...
    [1,1] * spanwise_location * wing.geometry.b/2 * sin(wing.geometry.v) ...
    ];

% compute center of pressure for lift due to camber
wing.x_cp0_camber_wing = - wing.xyz_wing_np(2,2)*sin(wing.geometry.phi) - wing.geometry.c/4;

% roll rate reduction factor
wing = simpleWingSetRollRateRed( wing );

% yaw rate reduction factor
wing = simpleWingSetYawRateRed( wing );

end