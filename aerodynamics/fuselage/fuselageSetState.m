function fuselage = fuselageSetState( fuselage, alpha, beta, V, omega, xyz_cg, varargin )
%fuselageSetState set state struct in fuselage struct
%   All required fuselage specifiy variables that change during simulation
%   are computed and the most important ones are stored in the
%   fuselage.state struct.
% 
% Syntax:
%   fuselage = fuselageSetState( fuselage, alpha, beta, V, omega, xyz_cg )
%   fuselage = fuselageSetState( fuselage, alpha, beta, V, omega, xyz_cg, ...
%       'atmosphere', atmosphere )
%   fuselage = fuselageSetState( fuselage, alpha, beta, V, omega, xyz_cg, ...
%       'wind', V_Wb, V_Wb_dt )
%   fuselage = fuselageSetState( fuselage, alpha, beta, V, omega, xyz_cg, ...
%       'structure_pos', structure_pos )
%   fuselage = fuselageSetState( fuselage, alpha, beta, V, omega, xyz_cg, ...
%       'structure_pos', structure_pos, 'structure_vel', structure_vel )
%   fuselage = fuselageSetState( fuselage, alpha, beta, V, omega, xyz_cg, ...
%       'unsteady', alpha_unst, beta_unst )
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   alpha           flight-path angle of attack (scalar), in rad
%   beta            flight-path sideslip angle (scalar), in rad
%   V               flight-path velocity (scalar), in m/s
%   omega           angular velocity (3x1 array), in rad/s
%   xyz_cg          center of gravity position in fuselage frame (3x1
%                   array), in m
%   atmosphere      atmosphere struct (see isaAtmosphere)
%   V_Wb            local wind velocity vecotrs
%                   (fuselage.state.external.V_Wb)
%   V_Wb_dt         local wind acceleration vectors
%                   (fuselage.state.external.V_Wb_dt)
%   structure_pos   structure position state
%   structure_vel   structure velocity state
%   alpha_unst      unsteady (effective) angle of attack
%                   (fuselage.state.aero.unsteady.alpha), in rad
%   beta_unst       unsteady (effective) sideslip angle
%                   (fuselage.state.aero.unsteady.beta), in rad
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageCreate, fuselageSetAeroState

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

for i = 1:length(varargin)
    if ischar(varargin{i})
        if isequal(varargin{i},'atmosphere')
            fuselage.state.external.atmosphere = varargin{i+1};
        elseif isequal(varargin{i},'V_Wb')
            fuselage.state.external.V_Wb(:) = varargin{i+1};
        elseif isequal(varargin{i},'V_Wb_dt')
            fuselage.state.external.V_Wb_dt(:) = varargin{i+1};
        elseif isequal(varargin{i},'structure_pos') && fuselage.config.is_flexible
            fuselage = fuselageSetGeometryState(fuselage,'pos',varargin{i+1});
        elseif isequal(varargin{i},'structure_vel') && fuselage.config.is_flexible
            fuselage = fuselageSetGeometryState(fuselage,'vel',varargin{i+1});
        elseif isequal(varargin{i},'unsteady') && fuselage.config.is_unsteady
            fuselage.state.aero.R_Ab_i(:) = varargin{i+1};
            fuselage.state.aero.R_Ab_i(1,:) = 0;
        end
    end
end

% set current rigid body state
fuselage = fuselageSetBodyState( fuselage, alpha, beta, V, omega );

% set aerodynamics state
fuselage = fuselageSetAeroState( fuselage, xyz_cg );

end