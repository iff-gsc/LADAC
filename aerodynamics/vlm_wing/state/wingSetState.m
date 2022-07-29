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
%                       'V_Wb'              Next variable is V_Wb,
%                       'V_Wb_dt'           Next variable is V_Wb_dt,
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
%   atmosphere          atmosphere struct (see isAtmosphere)
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

%% parse variable input arguments

for i = 1:length(varargin)
    if ischar(varargin{i})
        if isequal(varargin{i},'atmosphere')
            wing.state.external.atmosphere = varargin{i+1};
        elseif isequal(varargin{i},'V_Wb')
            wing.state.external.V_Wb(:) = varargin{i+1};
        elseif isequal(varargin{i},'V_Wb_dt')
            wing.state.external.V_Wb_dt(:) = varargin{i+1};
        elseif isequal(varargin{i},'structure_pos') && wing.config.is_flexible
            wing = wingSetGeometryState( wing, 'pos', varargin{i+1} );
        elseif isequal(varargin{i},'structure_vel') && wing.config.is_flexible
            wing = wingSetGeometryState( wing, 'vel', varargin{i+1} );
        elseif isequal(varargin{i},'unst_airfoil_state') && wing.config.is_unsteady
            wing.state.aero.unsteady.x(:) = varargin{i+1};
        elseif isequal(varargin{i},'dyn_stall_state') && wing.config.is_unsteady
            wing.state.aero.unsteady.X(:) = varargin{i+1};
        elseif isequal(varargin{i},'unst_flap_state') && wing.config.is_unsteady
            wing.state.aero.unsteady.z(:) = varargin{i+1};
        elseif isequal(varargin{i},'unst_act2_state') && wing.config.is_unsteady
            wing.state.aero.unsteady.z2(:) = varargin{i+1};
        elseif isequal(varargin{i},'tau_v') && wing.config.is_unsteady
            wing.state.aero.unsteady.tau_v(:) = varargin{i+1};
        elseif isequal(varargin{i},'alpha_ind') && wing.config.is_unsteady
            % feedback of last alpha_ind to speed up next iteration
            wing.state.aero.circulation.alpha_ind(:) = varargin{i+1};
        end
    end
end

%% set current wing state

% set current rigid body state
wing.state.body = wingSetBodyState( wing.state.body, alpha, beta, V, omega );

% actuator deflection
wing = wingSetActuators( wing, actuators_pos, actuators_rate );

% compute aerodynamic state
wing = wingSetAeroState( wing, pos_ref_c );

end

