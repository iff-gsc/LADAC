function fuselage = fuselageCreate( filename, n_sections, n_segments, varargin )
%fuselageCreate create fuselage struct from parameters file
% 
% Syntax:
%   fuselage = fuselageCreate( filename, n_sections, n_segments )
%   fuselage = fuselageCreate( filename, n_sections, n_segments, 'flexible', structure )
% 
% Inputs:
%   params_file    	file name of the parameters file (string), see
%                  	fuselage_params_default.m
%   n_sections      number of sections specified in the params_file (scalar)
%   n_segments      number of segments for the discretized aerodynamics
%                   model (scalar)
%   flag            Indicates that the next input is a specific variable
%                   that can be passed optionally:
%                       'flexible'          Next variable is structure.
%   structure     	a structure struct (see structureCreateFromNastran)
%                  	that must have nodes at similar positions at the
%                  	configured fuselage (else the coupling will not work).
%                  	This will enable flexible/aeroelastic computations.
%                  	By default the wing is rigid.
% 
% Outputs:
%   fuselage        fuselage struct (see fuselageInit)
% 
% Example:
%   fuselage = fuselageCreate( 'fuselage_params_default', 5, 20 )
% 
% See also:
%   fuselagePlotGeometry, fuselageSetState
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-2.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

% default parameters
is_flexible = false;
is_unsteady = false;
n_structure_states = 1;

% set user parameters
for i = 1:length(varargin)
    if ~ischar(varargin{i})
        continue;
    end
    switch varargin{i}
        case 'flexible'
            if isstruct(varargin{i+1})
                structure = varargin{i+1};
                is_flexible = true;
                n_structure_states = size(structure.K,1);
            else
                error('Invalid option for parameter flexible.')
            end
        case 'unsteady'
            is_unsteady = true;
    end
end

% init
fuselage = fuselageInit( n_sections, n_segments, n_structure_states );

fuselage = fuselageSetParams( fuselage, filename );

fuselage = fuselageSetGeometry( fuselage, n_segments );

fuselage.state.geometry = fuselage.geometry;

if is_flexible
    fuselage = fuselageSetAeroelasticity(fuselage,structure,true);
    fuselage.config.is_flexible = true;
end

if is_unsteady
    fuselage.config.is_unsteady(:) = true;
end

end