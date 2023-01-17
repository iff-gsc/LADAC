function fuselage = fuselageCreateWithCpacs( tigl_handle, fuse_UID, ...
    axis_reversed, n_segments, varargin )
%fuselageCreateWithCpacs create fuselage struct from CPACS file
% 
% Syntax:
%   fuselage = fuselageCreate( tigl_handle, n_sections, n_segments )
%   fuselage = fuselageCreate( tigl_handle, n_sections, n_segments, 'unsteady' )
%   fuselage = fuselageCreate( tigl_handle, n_sections, n_segments, 'flexible', structure )
% 
% Inputs:
%   params_file    	tigl_handle of the CPACS file (string) (see
%                   tiglOpenCPACSConfigurationTry)
%   fuse_UID        UID of the fuselage inside the CPACS file (string)
%   axis_reversed   3x1 array with 1 or -1; [-1;1;-1] means the the x-axis
%                   and the z-axis are reversed in CPACS compared the the
%                   flight dynamics body fixed axis
%   n_segments      number of segments for the discretized aerodynamics
%                   model (scalar)
%   'flexible'      Configurates a flexible fuselage; next variable must be
%                   structure (default: rigid fuselage).
%   'unsteady'      Configurates that the aerodynamics are treated as
%                   unsteady (default: steady aerodynamics).
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
%   fuselagePlotGeometry, fuselageSetState, fuselageCreate
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
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
fuselage = fuselageInit( n_segments, n_segments, n_structure_states );

% hard code some parameters
fuselage.params.C_D0 = 0.02;
fuselage.params.C_L_alpha = 0.2;
fuselage.params.is_straight = true;

fuselage = fuselageSetGeometryFromCpacs( fuselage, tigl_handle, fuse_UID, axis_reversed );

fuselage.state.geometry = fuselage.geometry;

if is_flexible
    fuselage = fuselageSetAeroelasticity(fuselage,structure,true);
    fuselage.config.is_flexible = true;
end

if is_unsteady
    fuselage.config.is_unsteady(:) = true;
end

end