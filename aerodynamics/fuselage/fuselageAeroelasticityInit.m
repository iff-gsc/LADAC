function aeroelasticity = fuselageAeroelasticityInit( n_segments, n_structure_states )
%fuselageAeroelasticityInit initializes/defines the aeroelasticity struct
%inside a fuselage struct.
% 
% Inputs:
%   n_segments          number of fuselage segments
%   n_structure_state   number of structure states
%
% Outputs:
%   aeroelasticity      aeroelasticity struct as defined by this function
% 
% Authors:
%   Yannic Beyer
% 
% See also:
%   fuselageSetAeroelasticity
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% note that control points are considered to be a vector as follows:
% control_points = [ x_1; y_1; z_1; x_2; y_2; z_2; x_3; ... ]

% interpolation matrix (T) from structure state (s) to Delta control points
% state (c) so that Delta_control_points = T_cs * structure_state
aeroelasticity.T_cs = zeros( 3*n_segments, n_structure_states );
% interpolation matrix (T) from structure state (s) to Delta angle of
% attack (a) so that Delta_alpha = T_as * structure_state
aeroelasticity.T_as = zeros( n_segments, n_structure_states );
% interpolation matrix (T) from structure (s) to Delta sideslip angle (B)
% so that Delta_beta = T_Bs * structure_state
aeroelasticity.T_Bs = zeros( n_segments, n_structure_states );
% interpolation matrix (T) from Delta control points (c) to structure state
% (s) so that structure_state = T_sc * Delta_control_points
aeroelasticity.T_sc = zeros( n_structure_states, 3*n_segments );

end

