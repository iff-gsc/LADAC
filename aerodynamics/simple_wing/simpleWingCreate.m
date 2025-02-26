function wing = simpleWingCreate( filename_wing, filename_airfoil, varargin )
% simpleWingCreate create simple wing struct with help of VLM
% 
% Syntax:
%   wing = simpleWingCreate( filename_wing, filename_airfoil )
% 
% Inputs:
%   filename_wing       File name of the parameters file (string), see
%                       wing_params_default.m
%   filename_airfoil    File name of airfoil parameters including stall
%                       (string), see simple_wing_params_default.m (the
%                       geometrical parameters will be ignored and will be
%                       taken from the filename_wing)
%   Name                Name of Name-Value Arguments:
%                           - 'Unsteady': define if unsteady aerodynamics
%                               should be considered
%                               (0: no, 1: yes), default: 0
%   Value               Value of Name-Value Arguments (see input Name)
% 
% Outputs:
%   wing                Simple wing struct, as defined by this function or
%                       by simpleWingLoadParams.m
% 
% See also:
%   simpleWingLoadParams, wingCreate, simpleWingGetDerivs, wingGetDerivs

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2024 Yannic Beyer
%   Copyright (C) 2024 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

is_unsteady = 0;
for i = 1:length(varargin)
    if strcmp(varargin{i},'Unsteady')
        is_unsteady(:) = varargin{i+1};
    end
end

vlm_wing = wingCreate( filename_wing, 40 );

[derivs,derivs2] = wingGetDerivs(vlm_wing,'CG',[-1/4*vlm_wing.params.c(1);0;0]);

% load parameters from file
run(filename_airfoil);

wing.is_unsteady = is_unsteady;


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
wing.geometry.phi = atan((x_0-x_end)/(y_end-y_0));
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
    eta_np = derivs2.eta_NP;
    x_np = derivs2.x_NP;
    wing.xyz_wing_np = [x_np*[1,1];eta_np*wing.geometry.b/2*[-1,1];0,0];

    % y position shift of the neutral point of the left wing side and the right
    % wing side in wing frame if beta=90deg, m
    wing.delta_y_np_a0b90_b = 0.90*wing.geometry.b/2;
    wing.delta_x_np_a0b90_b = 0;


    % compute control point location for local airspeed vector computation
    eta_p = sqrt(2*derivs.P(4)/derivs.alpha(3));
    spanwise_location = eta_p*eta_p/eta_np;
    x_c = derivs.Q(3)/-derivs.alpha(3);

    wing.C_m_dOmega_0 = derivs.Q(5) - -derivs.Q(3)*(x_np/wing.geometry.c);

    wing.xyz_cntrl_pt_wing = [ ...
        [1,1] * x_c * wing.geometry.c; ...
        [-1,1] * spanwise_location * wing.geometry.b/2; ...
        [1,1] * spanwise_location * wing.geometry.b/2 * sin(wing.geometry.v) ...
        ];

    % compute center of pressure for lift due to camber
    if isfield(wing,'c_m0')
        xi_cp_camber = divideFinite(-wing.c_m0,wing.polar.params.C_L0);
    else
        xi_cp_camber = 0.25;
    end
    wing.x_cp0_camber_wing = -xi_cp_camber*wing.geometry.c;
    
    wing.flap.dalpha_deta = 2 * derivs.eta(3,:)/derivs.alpha(3);
    wing.flap.y_cp_wing = 2 * derivs.eta(4,:)./derivs.eta(3,:) * wing.geometry.b/2;
    wing.flap.x_cp0_wing = derivs.eta(5,:)./-derivs.eta(3,:) * wing.geometry.c;
    wing.flap.lambda_K = max(0,mean(vlm_wing.params.flap_depth(vlm_wing.params.flap_depth~=0)));
    
    
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