function fuselagePlotEigenmode(fuselage,structure,num_eigenmode,varargin)
% fuselagePlotEigenmode visualize eigenmode of fuselage
% 
% Syntax:
%   fuselagePlotEigenmode(fuselage,structure,num_eigenmode)
%   fuselagePlotEigenmode(fuselage,structure,num_eigenmode,factor)
% 
% Inputs:
%   fuselage        fuselage struct (see fuselageInit)
%   structure     	a structure struct (see structureCreateFromNastran)
%   num_eigenmode   number of eigenmode to be visualized
%   factor          factor to increase the deformation (default is 1)
% 
% See also:
%   fuselageInit, fuselageCreate, fuselageCreateWithCpacs
% 

% Disclamer:
%   SPDX-License-Identifier: GPL-3.0-only
% 
%   Copyright (C) 2020-2022 Yannic Beyer
%   Copyright (C) 2022 TU Braunschweig, Institute of Flight Guidance
% *************************************************************************

if num_eigenmode > size(fuselage.aeroelasticity.T_cs,2)
    error('Requested eigenmode exceeds the number of eigenmodes.')
end

scaling = 1;
for i = 1:length(varargin)
    if strcmp(varargin{i},'Scaling')
        scaling = varargin{i+1};
    end
end

modal_state = pinv(structure.modal.T) * structure.modal.T(:,num_eigenmode+6) * scaling;
fuselage_deform = fuselageSetGeometryState( fuselage, 'pos', modal_state );
fuselage_deform.geometry = fuselage_deform.state.geometry;

fuselagePlotGeometry( fuselage, varargin{:}, 'LineColor', [0.5,0.5,0.5] );

hold on

fuselagePlotGeometry( fuselage_deform, varargin{:} );

end