function wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, pos_ref_c, varargin )
%wingSetState sets the state struct of a wing struct (wing.state).
% 
% Inputs:
%   wing            wing struct, see wingCreate
%   alpha           flight path angle of attack (scalar), in deg
%   beta            flight path sideslip angle (scalar), in deg
%   omega           kinematic angular velocity (3x1 array), in rad/s
%   actuators_pos   positions of 1st and 2nd actuator (2xN array), usually
%                   in deg
%   actuators_rate  rate of change of actuators positions (2xN array),
%                   usually in deg/s
%   xyz_cg          vehicle center of gravity position in wing coordinate
%                   system (3x1 array), in m
%   flag            Indicates that the next input is a specific variable
%                   that can be passed optionally:
%                       'atmosphere'        Next variable is atmosphere,
%                       'wind'              Next variables are V_Wb, V_Wb_dt,
%                       'structure_pos'     Next variable is structure_pos,
%                       'structure_vel'     Next variable is structure_vel,
%                       'unst_aero_state'   Next variable is unst_aero_state,
%                       'dyn_stall_state'   Next variable is dyn_stall_state,
%                       'unst_flap_state'   Next variable is unst_flap_state,
%                       'unst_act2_state'   Next variable is unst_act2_state,
%                       'tau_v'             Next variable is tau_v,
%                       'Delta_alpha'       Next variable is Delta_alpha,
%                       'alpha_ind'         Next variable is alpha_ind.
% 
%   atmosphere          atmosphere struct (see isaAtmosphere)
%   V_Wb                wind velocity at each control point
%                       (wing.state.external.V_Wb)
%   V_Wb_dt             wind acceleration at each control point
%                       (wing.state.external.V_Wb_dt)
%   structure_pos       position of structure (size(wing.T_vs,2)x1 array),
%                       usually in modal space (unit unknown)
%   structure_vel       velocity of structure (size(wing.T_vs,2)x1 array),
%                       usually in modal space (unit unknown)
%   unst_aero_state     unsteady airfoil state per segment
%                       (wing.state.aero.unsteady.x)
%   dyn_stall_state     dynamic stall state per segment
%                       (wing.state.aero.unsteady.X)
%   unst_flap_state     unsteady flap state per segment
%                       (wing.state.aero.unsteady.z)
%   unst_act2_state     unsteady 2nd actuator state per segment
%                       (wing.state.aero.unsteady.z2)
%   tau_v               dimensionless time of leading edge stall for
%                       dynamic stall (wing.state.aero.unsteady.tau_v)
%   alpha_ind           induced angle of attack to speed up VLM iteration
%                       (wing.state.aero.circulation.alpha_ind)
% 
% Outputs:
%   wing            wing struct, see wingCreate
% 
% Syntax:
%   % default:
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg )
%   % pass rigid body rotational acceleration:
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'omega_dt', omega_dt )
%   % pass atmosphere:
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'atmosphere', atmosphere )
%   % pass wind:
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'wind', V_Wb, V_Wb_dt )
%   % pass position of structure (usually modal):
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'structure_pos', structure_pos )
%   % pass velocity of structure (usually modal):
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'structure_vel', structure_vel )
%   % pass unsteady airfoil state (potential flow):
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'unst_aero_state', unst_aero_state )
%   % pass dynamic stall state:
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'dyn_stall_state', dyn_stall_state )
%   % pass unsteady state of flap:
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'unst_flap_state', unst_flap_state )
%   % pass unsteady state of 2nd actuator:
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'unst_act2_state', unst_act2_state )
%   % pass dimensionless time of leading edge vortex (dynamic stall):
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'tau_v', tau_v )
%   % pass induced angle of attack (speed up viscous airfoil correction):
%   wing = wingSetState(wing, alpha, beta, V, omega, actuators_pos, actuators_rate, xyz_cg, 'alpha_ind', alpha_ind )
% 
% See also: wingCreate, wingSetAeroState

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

%% decode variable input arguments

is_structure_state  = false;
is_unst_airfoil_state = false;
is_dyn_stall_state  = false;
is_unst_flap_state  = false;
is_unst_act2_state  = false;
is_tau_v_state      = false;
is_alpha_ind_fb     = false;

atmosphere      = isAtmosphere(0);

V_Wb            = zeros( size(wing.state.external.V_Wb) );
V_Wb_dt         = zeros( size(wing.state.external.V_Wb_dt) );

structure_pos   = zeros( size(wing.aeroelasticity.T_vs,2), 1 );
structure_vel   = zeros( size(wing.aeroelasticity.T_vs,2), 1 );

unst_aero_state = zeros(size(wing.state.aero.unsteady.x));
dyn_stall_state = zeros(size(wing.state.aero.unsteady.X));
unst_flap_state = zeros(size(wing.state.aero.unsteady.z));
unst_act2_state = zeros(size(wing.state.aero.unsteady.z2));
tau_v           = zeros(size(wing.state.aero.unsteady.tau_v));

alpha_ind       = zeros(size(wing.state.aero.circulation.alpha_ind));

for i = 1:length(varargin)
    if strcmp(varargin{i},'atmosphere')
        atmosphere = varargin{i+1};
    elseif strcmp(varargin{i},'wind')
        % idx_max: codegen workaround (yes, either me or Matlab is dumb)
        idx_min = min(numel(V_Wb),numel(varargin{i+1}));
        V_Wb(1:idx_min) = varargin{i+1}(1:idx_min);
        idx_min = min(numel(V_Wb_dt),numel(varargin{i+2}));
        V_Wb_dt(1:idx_min) = varargin{i+2}(1:idx_min);
    elseif strcmp(varargin{i},'structure_pos') && wing.config.is_flexible
        idx_min = min(numel(structure_pos),numel(varargin{i+1}));
        structure_pos(1:idx_min) = varargin{i+1}(1:idx_min);
        is_structure_state(:) = true;
    elseif strcmp(varargin{i},'structure_vel') && wing.config.is_flexible
        idx_min = min(numel(structure_vel),numel(varargin{i+1}));
        structure_vel(1:idx_min) = varargin{i+1}(1:idx_min);
    elseif strcmp(varargin{i},'unst_airfoil_state')
        idx_min = min(numel(unst_aero_state),numel(varargin{i+1}));
        unst_aero_state(1:idx_min) = varargin{i+1}(1:idx_min);
        is_unst_airfoil_state(:) = true;
    elseif strcmp(varargin{i},'dyn_stall_state')
        idx_min = min(numel(dyn_stall_state),numel(varargin{i+1}));
        dyn_stall_state(1:idx_min) = varargin{i+1}(1:idx_min);
        is_dyn_stall_state(:) = true;
    elseif strcmp(varargin{i},'unst_flap_state')
        idx_min = min(numel(unst_flap_state),numel(varargin{i+1}));
        unst_flap_state(1:idx_min) = varargin{i+1}(1:idx_min);
        is_unst_flap_state(:) = true;
    elseif strcmp(varargin{i},'unst_act2_state')
        idx_min = min(numel(unst_act2_state),numel(varargin{i+1}));
        unst_act2_state(1:idx_min) = varargin{i+1}(1:idx_min);
        is_unst_act2_state(:) = true;
    elseif strcmp(varargin{i},'tau_v')
        idx_min = min(numel(tau_v),numel(varargin{i+1}));
        tau_v(1:idx_min) = varargin{i+1}(1:idx_min);
        is_tau_v_state(:) = true;
    elseif strcmp(varargin{i},'alpha_ind')
        idx_min = min(numel(alpha_ind),numel(varargin{i+1}));
        alpha_ind(1:idx_min) = varargin{i+1}(1:idx_min);
        is_alpha_ind_fb(:) = true;
    end
end

%% set current wing state

% set current rigid body state
wing.state.body = wingSetBodyState( wing.state.body, alpha, beta, V, omega );

% set external state
wing.state.external = wingSetExternal( wing.state.external, V_Wb, V_Wb_dt, ...
    atmosphere );

% set flexible wing state
if wing.config.is_flexible
    if is_structure_state
        wing = wingSetGeometryState( wing, structure_pos, ...
            'structure_vel', structure_vel );
    else
        error('Structure state was not set.')
    end
end

% actuator deflection
wing = wingSetActuators( wing, actuators_pos, actuators_rate );

% unsteady aerodynamics states
if wing.config.is_unsteady
    if is_unst_airfoil_state
        wing.state.aero.unsteady.x = unst_aero_state;
    else
        error('Unsteady airfoil state was not specified although it was configured.')
    end
    if is_dyn_stall_state
        wing.state.aero.unsteady.X = dyn_stall_state;
    end
    if is_unst_flap_state
        wing.state.aero.unsteady.z = unst_flap_state;
    end
    if is_unst_act2_state
        wing.state.aero.unsteady.z2 = unst_act2_state;
    end
    if is_tau_v_state
        wing.state.aero.unsteady.tau_v = tau_v;
    end
end

% feedback of last alpha_ind to speed up next iteration
if is_alpha_ind_fb
    wing.state.aero.circulation.alpha_ind = alpha_ind;
end

% compute aerodynamic state
wing = wingSetAeroState( wing, pos_ref_c );

end

