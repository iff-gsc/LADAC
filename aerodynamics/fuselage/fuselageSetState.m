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
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   alpha           flight-path angle of attack (scalar), in rad
%   beta            flight-path sideslip angle (scalar), in rad
%   V               flight-path velocity (scalar), in m/s
%   omega           angular velocity (3x1 array), in rad/s
%   xyz_cg          center of gravity position in fuselage frame (3x1
%                   array), in m
%   atmosphere      atmosphere struct (see isAtmosphere)
%   V_Wb            local wind velocity vecotrs
%                   (fuselage.state.external.V_Wb)
%   V_Wb_dt         local wind acceleration vectors
%                   (fuselage.state.external.V_Wb_dt)
%   structure_pos   structure position state
%   structure_vel   structure velocity state
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% See also:
%   fuselageInit, fuselageCreate, fuselageSetAeroState
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

is_structure_state  = false;
is_structure_vel    = false;
is_unsteady         = false;

V_Wb            = zeros( size(fuselage.state.external.V_Wb) );
V_Wb_dt         = zeros( size(fuselage.state.external.V_Wb_dt) );
atmosphere      = isAtmosphere(0);
structure_pos   = zeros( size(fuselage.aeroelasticity.T_cs,2), 1 );
structure_vel   = zeros( size(fuselage.aeroelasticity.T_cs,2), 1 );
alpha_unst      = zeros( size(fuselage.state.aero.unsteady.alpha) );
beta_unst       = zeros( size(fuselage.state.aero.unsteady.beta) );

for i = 1:length(varargin)
    if strcmp(varargin{i},'atmosphere')
        atmosphere = varargin{i+1};
    elseif strcmp(varargin{i},'wind')
        idx_min = min(numel(V_Wb),numel(varargin{i+1}));
        V_Wb(1:idx_min) = varargin{i+1}(1:idx_min);
        idx_min = min(numel(V_Wb_dt),numel(varargin{i+2}));
        V_Wb_dt(1:idx_min) = varargin{i+2}(1:idx_min);
    elseif strcmp(varargin{i},'structure_pos') && fuselage.config.is_flexible
        idx_min = min(numel(structure_pos),numel(varargin{i+1}));
        structure_pos(1:idx_min) = varargin{i+1}(1:idx_min);
        is_structure_state(:) = true;
    elseif strcmp(varargin{i},'structure_vel')
        idx_min = min(numel(structure_vel),numel(varargin{i+1}));
        structure_vel(1:idx_min) = varargin{i+1}(1:idx_min);
        is_structure_vel(:) = true;
    elseif strcmp(varargin{i},'unsteady')
        idx_min = min(numel(alpha_unst),numel(varargin{i+1}));
        alpha_unst(1:idx_min) = varargin{i+1}(1:idx_min);
        idx_min = min(numel(beta_unst),numel(varargin{i+1}));
        beta_unst(1:idx_min) = varargin{i+2}(1:idx_min);
        is_unsteady(:) = true;
    end
end

% set current rigid body state
fuselage = fuselageSetBodyState( fuselage, alpha, beta, V, omega );

% set external state
fuselage = fuselageSetExternal( fuselage, V_Wb, V_Wb_dt, atmosphere );

% set flexible geometry (to do)
if is_structure_state && is_structure_vel
    fuselage = fuselageSetGeometryState( fuselage, structure_pos, 'structure_vel', structure_vel );
elseif is_structure_state && ~is_structure_vel
    fuselage = fuselageSetGeometryState( fuselage, structure_pos );
end

if is_unsteady
    fuselage = fuselageSetUnstAeroState( fuselage, alpha_unst, beta_unst );
end

% set aerodynamics state
fuselage = fuselageSetAeroState( fuselage, xyz_cg );

end