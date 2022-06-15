function wing = wingSetGeometryState( wing, ...
    modal_pos_state, varargin ) 
% Description of wingSetGeometryState

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

is_structure_vel = false;

for i = 1:length(varargin)
    if strcmp(varargin{i},'structure_vel')
        modal_vel_state = varargin{i+1};
        is_structure_vel = true;
    end
end

% displacements
geometry_v = wing.aeroelasticity.T_vs * modal_pos_state;
geometry_c = wing.aeroelasticity.T_cs * modal_pos_state;

% assign to struct
wing.state.geometry.vortex = wingSetPosition( wing.state.geometry.vortex, geometry_v, 3 );
wing.state.geometry.ctrl_pt = wingSetPosition( wing.state.geometry.ctrl_pt, geometry_c, 4 );

if is_structure_vel
    % time derivative of displacements
    geometry_c_dt = wing.aeroelasticity.T_cs * modal_vel_state;
    wing.state.geometry.ctrl_pt_dt = wingSetPosition( wing.state.geometry.ctrl_pt, geometry_c_dt, 4, true );
end


end