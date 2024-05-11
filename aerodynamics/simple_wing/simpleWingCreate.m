function wing = simpleWingCreate( filename_wing, filename_airfoil )

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

vlm_wing = wingCreate( filename_wing, 40 );

[derivs,derivs2] = wingGetDerivs(vlm_wing,'CG',[-1/4*vlm_wing.params.c(1);0;0]);

% load parameters from file
run(filename_airfoil);



% span, m
wing.geometry.b = vlm_wing.params.b;
% reference surface, m^2
wing.geometry.S = vlm_wing.params.S;
% mean chord
wing.geometry.c = wing.geometry.S / wing.geometry.b;
% Oswald's efficiency factor, - (hard coded, to do)
wing.geometry.e = 0.8;
% sweep, rad
x_0 = vlm_wing.geometry.line_25.pos(1,floor(end/2)+1);
x_end = vlm_wing.geometry.line_25.pos(1,end);
y_0 = vlm_wing.geometry.line_25.pos(2,floor(end/2)+1);
y_end = vlm_wing.geometry.line_25.pos(2,end);
wing.geometry.phi = atan((x_end-x_0)/(y_end-y_0));
% dihedral angle, rad
z_0 = vlm_wing.geometry.line_25.pos(3,floor(end/2)+1);
z_end = vlm_wing.geometry.line_25.pos(3,end);
wing.geometry.v = atan((z_end-z_0)/(y_end-y_0));

% add potentially missing flap scaling parameters (deactivate scaling by
% default)
if ~isfield(wing.flap,'eff_scaling')
    wing.flap.eff_scaling = 1;
    wing.flap.is_eff_scaling = 0;
end

C_Lalpha_des = -derivs.alpha(3);
wing.polar.params.C_Lalpha = C_Lalpha_des;

num_flaps = length(vlm_wing.state.actuators.pos);

while true

    % roll rate reduction factor
    wing.roll_rate_red_factor = 1;

    % yaw rate reduction factor
    wing.yaw_rate_red_factor = 1;

    % compute elevon/flap effectiveness
    wing = simpleWingSetFlapEffectiveness( wing );

    wing = simpleWingSetInducedDragFactor( wing );
    
    % set aerodynamic polar map
    wing = simpleWingSetLiftCurve( wing );
    wing = simpleWingSetDragCurve( wing );

    % position of the neutral point in the wing frame, m
    eta_np = sqrt(2*derivs.p(4)/derivs.alpha(3));
    x_np = derivs2.x_NP;
    wing.xyz_wing_np = [x_np*[1,1];eta_np*wing.geometry.b/2*[-1,1];0,0];

    % y position shift of the neutral point of the left wing side and the right
    % wing side in wing frame if beta=90deg, m
    wing.delta_y_np_a0b90_b = 0.90*wing.geometry.b/2;
    wing.delta_x_np_a0b90_b = 0;


    % compute control point location for local airspeed vector computation
    spanwise_location = eta_np;
    x_c = derivs.q(3)/-derivs.alpha(3);

    wing.C_m_dOmega_0 = derivs.q(5) - -derivs.q(3)*(x_np/wing.geometry.c);

    wing.xyz_cntrl_pt_wing = [ ...
        [1,1] * x_c * wing.geometry.c; ...
        [-1,1] * spanwise_location * wing.geometry.b/2; ...
        [1,1] * spanwise_location * wing.geometry.b/2 * sin(wing.geometry.v) ...
        ];

    % compute center of pressure for lift due to camber
    wing.x_cp0_camber_wing = - wing.xyz_wing_np(2,2)*sin(wing.geometry.phi) - wing.geometry.c/4;
    
    wing.flap.dalpha_deta = 2 * derivs.eta(3,:)/derivs.alpha(3);
    wing.flap.y_cp_wing = 2 * derivs.eta(4,:)./derivs.eta(3,:) * wing.geometry.b/2;
    wing.flap.x_cp0_wing = derivs.eta(5,:)./-derivs.eta(3,:) * wing.geometry.c;
    
    
    derivs_simple = simpleWingGetDerivs(wing);
    C_Lalpha = -derivs_simple.alpha(3);
    err = C_Lalpha_des - C_Lalpha;
    if abs(err) < 1e-6
        break;
    else
        wing.polar.params.C_Lalpha = wing.polar.params.C_Lalpha + err;
    end

end

end