function fuselage = fuselageInit( n_sections, n_segments, n_structure_states )
% fuselageInit define and initialize fuselage struct.
%   With the fuselage struct, aerodynamic and aeroelastic computations of
%   fuselages can be performed.
%   The fuselage struct contains all important input and output variables
%   as well as important intermediate results.
% 
% Inputs:
%   n_sections          number of sections specified in the params_file (scalar)
%   n_segments          number of segments for the discretized aerodynamics
%                       model (scalar)
%   n_structure_states  number of structure states
% 
% Outputs:
%   fuselage            fuselage struct as defined by this function
% 
% See also:
%   fuselageGeometryInit, fuselageStateInit

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% fuselage parameters struct
fuselage.params = fuselageParamsInit( n_sections );
% number of segments for the discretized aerodynamics model
fuselage.n_segments = n_segments;
% fuselage geometry struct
fuselage.geometry = fuselageGeometryInit( n_segments );
% wing influence on the fuselage (to do)
fuselage.wing_influence = zeros( 1, n_segments );
% fuselage state struct
fuselage.state = fuselageStateInit( n_segments, fuselage.geometry );
% minimum size aeroelastic parameters must be initialized for Simulink
if n_structure_states < 1
    n_structure_states = 1;
end
% fuselage aeroelasticity struct
fuselage.aeroelasticity = fuselageAeroelasticityInit( n_segments, n_structure_states );
% logical whether the fuselage is considered to be flexible or not
fuselage.config.is_flexible = false;
fuselage.config.is_unsteady = false;

end