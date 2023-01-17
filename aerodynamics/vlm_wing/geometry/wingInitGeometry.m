function geometry = wingInitGeometry( n_panel )
% wingInitGeometry initializes a geometry struct.
%
% Inputs:
% 	 params                 A struct containing all parameters and key
%                           characteristics of the wing
%                           (struct, see wingLoadParameters)
%    n_panel                The number of panels into which the wing is
%                           divided (double)
%
% Outputs:
%    geometry               A struct which contains all the computed
%                           geometry values and postions for the panels and
%                           vortexes (struct as defined by this function)
%
% See also: wingCreate
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************


geometry.origin = zeros(3,1);
geometry.rotation = zeros(3,1);

geometry.vortex.pos = zeros(3,n_panel+1);
geometry.vortex.c = zeros(1,n_panel+1);

geometry.ctrl_pt.pos = zeros(3,n_panel);
geometry.ctrl_pt.c = zeros(1,n_panel);
geometry.ctrl_pt.local_incidence = zeros(1,n_panel);

geometry.segments.control_input_index_local = zeros(2,n_panel);
geometry.segments.type_local = zeros(1,n_panel);
geometry.segments.flap_depth = zeros(1,n_panel);

end
