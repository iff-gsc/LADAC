function geometry = fuselageGeometryInit( n_segments )
%fuselageGeometryInit define and initialize geometry struct for fuselage
% struct
%   The geometry struct contains information about the fuselage geometry
%   depending on the specified discretization.
% 
% Inputs:
%   n_segments      number of segments for the discretized aerodynamics
%                   model (scalar)
% 
% Outputs:
%   geometry        geometry struct as defined by this function

% Disclaimer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% control point positions of the discretized fuselage, in m
geometry.cntrl_pos = zeros( 3, n_segments );
% segments border positions of the discretized fuselage, in m
geometry.border_pos = zeros( 3, n_segments + 1 );
% fuselage width at each segment border, in m
geometry.width = zeros( 1, n_segments + 1 );
% modified fuselage width to allow lift generation (wider than width at the
% rear), in m
geometry.width_visc = zeros( 1, n_segments + 1 );
% angle of attack at each segment border, in rad
geometry.alpha = zeros( 1, n_segments );
% sideslip angle at each segment border, in rad
geometry.beta = zeros( 1, n_segments );

end

