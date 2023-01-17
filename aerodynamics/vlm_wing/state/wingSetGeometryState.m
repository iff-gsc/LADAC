function wing = wingSetGeometryState( wing, varargin ) 
% Description of wingSetGeometryState

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

for i = 1:length(varargin)
    if strcmp(varargin{i},'pos')
        % displacements
        geometry_v = wing.aeroelasticity.T_vs * varargin{i+1};
        geometry_c = wing.aeroelasticity.T_cs * varargin{i+1};
        % assign to struct
        wing.state.geometry.vortex = wingSetPosition( wing.state.geometry.vortex, geometry_v, 3 );
        wing.state.geometry.ctrl_pt = wingSetPosition( wing.state.geometry.ctrl_pt, geometry_c, 4 );
    elseif strcmp(varargin{i},'vel')
        % time derivative of displacements
        geometry_c_dt = wing.aeroelasticity.T_cs * varargin{i+1};
        wing.state.geometry.ctrl_pt_dt = wingSetPosition( wing.state.geometry.ctrl_pt, geometry_c_dt, 4, true );
    end
end

end